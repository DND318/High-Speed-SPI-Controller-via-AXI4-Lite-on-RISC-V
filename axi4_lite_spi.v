// MODULE: AXI4-Lite SPI/UART Dual-Mode Master
// Phiên bản: fixed (so với bản gốc dual-mode)
//
// Fixes đã áp dụng:
//   F1 – axl_uart_rx: thêm 2-FF synchronizer cho uart_rx (chống metastability)
//   F2 – axl_uart_tx: xfer_done dời về cuối S_STOP2 khi stop2=1
//              tránh CPU push byte mới trong khi stop bit 2 chưa phát xong
//   F3 – axl_uart_tx: thêm comment giải thích inter-frame gap timing
//
// Register Map (backward-compatible với SPI-only version):
//  0x00 R_CTRL  [0]=CPOL [1]=CPHA [2]=LSBF [3]=MODE [4]=PAR_EN [5]=PAR_ODD [6]=STOP2
//  0x04 R_BAUD  [15:0]=divisor   f_tick = f_clk/(div+1)
//                                SPI: f_sclk = f_tick/2
//                                UART: 1 bit = 1 tick period
//  0x08 R_TXDATA push TX FIFO   (UART dùng [7:0])
//  0x0C R_RXDATA pop  RX FIFO   (UART trả {24'b0, byte})
//  0x10 R_STATUS [0]=BUSY [1]=TX_FULL [2]=RX_NONEMPTY [3]=TX_EMPTY [4]=RX_FULL
//  0x14 R_IER    Interrupt Enable Register
//  0x18 R_ISR    Interrupt Status W1C
//               [0]=TX_EMPTY [1]=RX_NONEMPTY [2]=XFER_DONE [3]=TX_FULL
// =====================================================================
module axi4_lite_spi #(
    parameter AXI_ADDR_WIDTH = 8
)(
    input  wire                      aclk,
    input  wire                      aresetn,

    input  wire [AXI_ADDR_WIDTH-1:0] awaddr,
    input  wire                      awvalid,
    output wire                      awready,
    input  wire [31:0]               wdata,
    input  wire [3:0]                wstrb,
    input  wire                      wvalid,
    output wire                      wready,
    output wire [1:0]                bresp,
    output wire                      bvalid,
    input  wire                      bready,
    input  wire [AXI_ADDR_WIDTH-1:0] araddr,
    input  wire                      arvalid,
    output wire                      arready,
    output wire [31:0]               rdata,
    output wire [1:0]                rresp,
    output wire                      rvalid,
    input  wire                      rready,

    // SPI pins (idle khi MODE=UART: mosi=0, sclk=0, ss_n=1)
    output wire                      mosi,
    input  wire                      miso,
    output wire                      sclk,
    output wire                      ss_n,

    // UART pins (idle khi MODE=SPI: uart_tx=1/mark)
    output wire                      uart_tx,
    input  wire                      uart_rx,

    output wire                      irq_o
);

// ─────────────────────────────────────────────────────────────────────
// Register Addresses
// ─────────────────────────────────────────────────────────────────────
localparam R_CTRL   = 8'h00;
localparam R_BAUD   = 8'h04;
localparam R_TXDATA = 8'h08;
localparam R_RXDATA = 8'h0C;
localparam R_STATUS = 8'h10;
localparam R_IER    = 8'h14;
localparam R_ISR    = 8'h18;

// ─────────────────────────────────────────────────────────────────────
// AXI4-Lite Handshake (không thay đổi)
// ─────────────────────────────────────────────────────────────────────
reg axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
reg [31:0] axi_rdata;

assign awready = axi_awready; assign wready  = axi_wready;
assign bresp   = 2'b00;       assign bvalid  = axi_bvalid;
assign arready = axi_arready; assign rdata   = axi_rdata;
assign rresp   = 2'b00;       assign rvalid  = axi_rvalid;

always @(posedge aclk) begin
    if (!aresetn) begin axi_awready <= 1'b0; axi_wready <= 1'b0; end
    else begin
        if (~axi_awready && awvalid && wvalid) begin
            axi_awready <= 1'b1; axi_wready <= 1'b1;
        end else begin
            axi_awready <= 1'b0; axi_wready <= 1'b0;
        end
    end
end

wire slv_reg_wren = axi_wready && wvalid && axi_awready && awvalid;

always @(posedge aclk) begin
    if (!aresetn) axi_bvalid <= 1'b0;
    else if (slv_reg_wren && ~axi_bvalid) axi_bvalid <= 1'b1;
    else if (bready && axi_bvalid)        axi_bvalid <= 1'b0;
end

always @(posedge aclk) begin
    if (!aresetn) axi_arready <= 1'b0;
    else if (~axi_arready && arvalid) axi_arready <= 1'b1;
    else                              axi_arready <= 1'b0;
end

wire slv_reg_rden = axi_arready && arvalid && ~axi_rvalid;

always @(posedge aclk) begin
    if (!aresetn) axi_rvalid <= 1'b0;
    else if (slv_reg_rden)         axi_rvalid <= 1'b1;
    else if (axi_rvalid && rready) axi_rvalid <= 1'b0;
end

// ─────────────────────────────────────────────────────────────────────
// Registers & Control Decode
// ─────────────────────────────────────────────────────────────────────
reg [31:0] reg_ctrl, reg_baud, reg_ier;
reg [3:0]  reg_isr;

wire cpol         = reg_ctrl[0];
wire cpha         = reg_ctrl[1];
wire lsbf         = reg_ctrl[2];
wire mode_uart    = reg_ctrl[3];  // 0 = SPI, 1 = UART
wire uart_par_en  = reg_ctrl[4];
wire uart_par_odd = reg_ctrl[5];
wire uart_stop2   = reg_ctrl[6];

wire [15:0] baud_div = reg_baud[15:0];

// ─────────────────────────────────────────────────────────────────────
// Internal Signals
// ─────────────────────────────────────────────────────────────────────
wire baud_tick;
wire tx_fifo_empty, tx_fifo_full, rx_fifo_empty, rx_fifo_full;
wire [31:0] tx_fifo_dout, rx_fifo_dout;

// SPI engine
wire [31:0] spi_rx_data_w;
wire        spi_tx_rd_en_w, spi_rx_wr_en_w;
wire        spi_busy_w, spi_xfer_done_w;
wire        spi_mosi_w, spi_sclk_w, spi_ss_n_w;

// UART TX engine
wire        uart_tx_wire;
wire        uart_tx_rd_en_w;
wire        uart_tx_busy_w;
wire        uart_tx_done_w;

// UART RX engine
wire [7:0]  uart_rx_data_w;
wire        uart_rx_wr_en_w;

// ─────────────────────────────────────────────────────────────────────
// [F1 FIX] 2-FF Synchronizer cho uart_rx
// Ngăn metastability khi uart_rx bất đồng bộ với aclk
// ─────────────────────────────────────────────────────────────────────
reg uart_rx_s1, uart_rx_sync;   // 2 tầng FF
always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        uart_rx_s1   <= 1'b1;   // idle high
        uart_rx_sync <= 1'b1;
    end else begin
        uart_rx_s1   <= uart_rx;       // FF tầng 1
        uart_rx_sync <= uart_rx_s1;    // FF tầng 2 (output an toàn)
    end
end

// ─────────────────────────────────────────────────────────────────────
// Mode Mux
// ─────────────────────────────────────────────────────────────────────
wire        tx_rd_en    = mode_uart ? uart_tx_rd_en_w         : spi_tx_rd_en_w;
wire [31:0] rx_fifo_din = mode_uart ? {24'd0, uart_rx_data_w} : spi_rx_data_w;
wire        rx_wr_en    = mode_uart ? uart_rx_wr_en_w         : spi_rx_wr_en_w;
wire        busy_engine      = mode_uart ? uart_tx_busy_w  : spi_busy_w;
wire        xfer_done_engine = mode_uart ? uart_tx_done_w  : spi_xfer_done_w;

assign mosi    = mode_uart ? 1'b0         : spi_mosi_w;
assign sclk    = mode_uart ? 1'b0         : spi_sclk_w;
assign ss_n    = mode_uart ? 1'b1         : spi_ss_n_w;
assign uart_tx = mode_uart ? uart_tx_wire : 1'b1;

// ─────────────────────────────────────────────────────────────────────
// Register Write Logic
// ─────────────────────────────────────────────────────────────────────
reg tx_wr_en_reg;
reg tx_fifo_empty_r, rx_fifo_empty_r;

always @(posedge aclk) begin
    if (!aresetn) begin
        reg_ctrl        <= 32'd0;
        reg_baud        <= 32'd8;
        reg_ier         <= 32'd0;
        reg_isr         <= 4'd0;
        tx_wr_en_reg    <= 1'b0;
        tx_fifo_empty_r <= 1'b1;
        rx_fifo_empty_r <= 1'b1;
    end else begin
        tx_wr_en_reg    <= 1'b0;
        tx_fifo_empty_r <= tx_fifo_empty;
        rx_fifo_empty_r <= rx_fifo_empty;

        if (slv_reg_wren) begin
            case (awaddr[7:0])
                R_CTRL:   reg_ctrl <= wdata;
                R_BAUD:   reg_baud <= wdata;
                R_TXDATA: tx_wr_en_reg <= 1'b1;
                R_IER:    reg_ier  <= wdata;
                R_ISR:    reg_isr  <= reg_isr & ~wdata[3:0];
            endcase
        end

        if (tx_fifo_empty  && !tx_fifo_empty_r) reg_isr[0] <= 1'b1;
        if (!rx_fifo_empty && rx_fifo_empty_r)  reg_isr[1] <= 1'b1;
        if (xfer_done_engine)                   reg_isr[2] <= 1'b1;
        if (tx_fifo_full)                       reg_isr[3] <= 1'b1;
    end
end

// ─────────────────────────────────────────────────────────────────────
// Register Read Logic
// ─────────────────────────────────────────────────────────────────────
wire rx_rd_en_comb = slv_reg_rden && (araddr[7:0] == R_RXDATA);

always @(posedge aclk) begin
    if (!aresetn) axi_rdata <= 32'd0;
    else if (slv_reg_rden) begin
        case (araddr[7:0])
            R_CTRL:   axi_rdata <= reg_ctrl;
            R_BAUD:   axi_rdata <= reg_baud;
            R_RXDATA: axi_rdata <= mode_uart
                                   ? {24'd0, rx_fifo_dout[7:0]}
                                   : rx_fifo_dout;
            R_STATUS: axi_rdata <= {27'd0,
                                    rx_fifo_full,
                                    tx_fifo_empty,
                                    ~rx_fifo_empty,
                                    tx_fifo_full,
                                    busy_engine};
            R_IER:    axi_rdata <= reg_ier;
            R_ISR:    axi_rdata <= {28'd0, reg_isr};
            default:  axi_rdata <= 32'hDEAD_BEEF;
        endcase
    end
end

assign irq_o = |(reg_isr & reg_ier[3:0]);

// ─────────────────────────────────────────────────────────────────────
// Sub-modules
// ─────────────────────────────────────────────────────────────────────
axl_baud_div u_baud (
    .clk(aclk), .rstn(aresetn), .div(baud_div), .tick(baud_tick)
);

axl_fwft_fifo u_tx_fifo (
    .clk(aclk), .rstn(aresetn),
    .wr_en(tx_wr_en_reg), .din(wdata),
    .rd_en(tx_rd_en), .dout(tx_fifo_dout),
    .empty(tx_fifo_empty), .full(tx_fifo_full)
);

axl_fwft_fifo u_rx_fifo (
    .clk(aclk), .rstn(aresetn),
    .wr_en(rx_wr_en), .din(rx_fifo_din),
    .rd_en(rx_rd_en_comb), .dout(rx_fifo_dout),
    .empty(rx_fifo_empty), .full(rx_fifo_full)
);

axl_engine u_spi_engine (
    .clk(aclk), .rstn(aresetn & ~mode_uart),
    .tick(baud_tick), .cpol(cpol), .cpha(cpha), .lsbf(lsbf),
    .tx_data(tx_fifo_dout), .tx_empty(tx_fifo_empty | mode_uart),
    .tx_rd_en(spi_tx_rd_en_w),
    .rx_data(spi_rx_data_w), .rx_wr_en(spi_rx_wr_en_w),
    .xfer_done(spi_xfer_done_w),
    .mosi(spi_mosi_w), .miso(miso),
    .sclk(spi_sclk_w), .ss_n(spi_ss_n_w), .busy(spi_busy_w)
);

axl_uart_tx u_uart_tx (
    .clk(aclk), .rstn(aresetn & mode_uart),
    .tick(baud_tick),
    .tx_data(tx_fifo_dout[7:0]),
    .tx_empty(tx_fifo_empty | ~mode_uart),
    .tx_rd_en(uart_tx_rd_en_w),
    .uart_tx(uart_tx_wire),
    .busy(uart_tx_busy_w), .xfer_done(uart_tx_done_w),
    .parity_en(uart_par_en), .parity_odd(uart_par_odd), .stop2(uart_stop2)
);

axl_uart_rx u_uart_rx (
    .clk(aclk), .rstn(aresetn & mode_uart),
    .baud_div(baud_div),
    .uart_rx(uart_rx_sync),    // ← dùng uart_rx ĐÃ SYNC (F1 fix)
    .rx_data(uart_rx_data_w), .rx_wr_en(uart_rx_wr_en_w),
    .parity_en(uart_par_en), .parity_odd(uart_par_odd)
);

endmodule


// =====================================================================
// Baud Divider (không thay đổi)
// =====================================================================
module axl_baud_div (
    input  wire        clk, rstn,
    input  wire [15:0] div,
    output reg         tick
);
    reg [15:0] cnt;
    always @(posedge clk) begin
        if (!rstn) begin cnt <= 16'd0; tick <= 1'b0; end
        else begin
            tick <= 1'b0;
            if (div == 16'd0)      tick <= 1'b1;
            else if (cnt == 16'd0) begin cnt <= div; tick <= 1'b1; end
            else                   cnt  <= cnt - 1'b1;
        end
    end
endmodule


// =====================================================================
// FWFT FIFO (không thay đổi)
// =====================================================================
module axl_fwft_fifo #(
    parameter WIDTH = 32,
    parameter DEPTH = 16
)(
    input  wire              clk, rstn, wr_en,
    input  wire [WIDTH-1:0]  din,
    input  wire              rd_en,
    output wire [WIDTH-1:0]  dout,
    output wire              empty, full
);
    reg [WIDTH-1:0]         mem   [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] wptr, rptr;
    reg [$clog2(DEPTH):0]   count;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin wptr <= 0; rptr <= 0; count <= 0; end
        else begin
            if (wr_en && !full)  begin mem[wptr] <= din; wptr <= wptr + 1; end
            if (rd_en && !empty) rptr <= rptr + 1;
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1;
                2'b01: count <= count - 1;
                default: ;
            endcase
        end
    end
    assign empty = (count == 0);
    assign full  = (count == DEPTH);
    assign dout  = mem[rptr];
endmodule


// =====================================================================
// SPI Engine (không thay đổi)
// =====================================================================
module axl_engine (
    input  wire        clk, rstn, tick,
    input  wire        cpol, cpha, lsbf,
    input  wire [31:0] tx_data,
    input  wire        tx_empty,
    output reg         tx_rd_en,
    output reg  [31:0] rx_data,
    output reg         rx_wr_en, xfer_done,
    output wire        mosi,
    input  wire        miso,
    output reg         sclk, ss_n, busy
);
    localparam S_IDLE  = 3'd0;
    localparam S_SETUP = 3'd1;
    localparam S_PH1   = 3'd2;
    localparam S_PH2   = 3'd3;
    localparam S_HOLD  = 3'd4;

    reg [2:0]  state;
    reg [5:0]  bit_cnt;
    reg [31:0] sh_tx, sh_rx;

    assign mosi = lsbf ? sh_tx[0] : sh_tx[31];

    always @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE; ss_n <= 1'b1; sclk <= 1'b0; busy <= 1'b0;
            tx_rd_en <= 1'b0; rx_wr_en <= 1'b0; xfer_done <= 1'b0;
            bit_cnt <= 6'd0; sh_tx <= 32'd0; sh_rx <= 32'd0; rx_data <= 32'd0;
        end else begin
            tx_rd_en <= 1'b0; rx_wr_en <= 1'b0; xfer_done <= 1'b0;
            case (state)
                S_IDLE: begin
                    ss_n <= 1'b1; busy <= 1'b0; sclk <= cpol;
                    if (!tx_empty) begin
                        sh_tx <= tx_data; tx_rd_en <= 1'b1;
                        ss_n <= 1'b0; busy <= 1'b1; bit_cnt <= 6'd0;
                        state <= S_SETUP;
                    end
                end
                S_SETUP: if (tick) state <= S_PH1;
                S_PH1: if (tick) begin
                    sclk <= ~cpol;
                    if (!cpha) sh_rx <= lsbf ? {miso,sh_rx[31:1]} : {sh_rx[30:0],miso};
                    state <= S_PH2;
                end
                S_PH2: if (tick) begin
                    sclk <= cpol;
                    if (cpha) sh_rx <= lsbf ? {miso,sh_rx[31:1]} : {sh_rx[30:0],miso};
                    if (bit_cnt == 6'd31) begin
                        rx_data  <= cpha ? (lsbf ? {miso,sh_rx[31:1]} : {sh_rx[30:0],miso}) : sh_rx;
                        rx_wr_en <= 1'b1; xfer_done <= 1'b1;
                        state    <= S_HOLD;
                    end else begin
                        bit_cnt <= bit_cnt + 1;
                        sh_tx   <= lsbf ? {1'b0,sh_tx[31:1]} : {sh_tx[30:0],1'b0};
                        state   <= S_PH1;
                    end
                end
                S_HOLD: if (tick) begin ss_n <= 1'b1; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


// =====================================================================
// UART TX Engine [F2 FIX: xfer_done timing sửa cho stop2=1]
//
// Frame: [START=0] [D0..D7 LSB-first] [PARITY?] [STOP1=1] [STOP2?=1]
//
// Timing note:
//   S_IDLE → S_START không chờ tick → có thể trễ tối đa 1 baud period
//   trước start bit (inter-frame gap). Đây là hành vi bình thường UART.
//
// F2 Fix: xfer_done chỉ set khi frame THỰC SỰ kết thúc:
//   - stop2=0: set tại S_STOP1
//   - stop2=1: set tại S_STOP2 (STOP2 hoàn tất rồi mới báo done)
// =====================================================================
module axl_uart_tx (
    input  wire       clk, rstn, tick,
    input  wire [7:0] tx_data,
    input  wire       tx_empty,
    output reg        tx_rd_en,
    output reg        uart_tx,
    output reg        busy,
    output reg        xfer_done,
    input  wire       parity_en, parity_odd, stop2
);
    localparam S_IDLE  = 3'd0;
    localparam S_START = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_PAR   = 3'd3;
    localparam S_STOP1 = 3'd4;
    localparam S_STOP2 = 3'd5;

    reg [2:0] state, bit_cnt;
    reg [7:0] sh_tx;
    reg       par_bit;

    always @(posedge clk) begin
        if (!rstn) begin
            state    <= S_IDLE; uart_tx <= 1'b1; busy <= 1'b0;
            tx_rd_en <= 1'b0;  xfer_done <= 1'b0;
            bit_cnt  <= 3'd0;  sh_tx <= 8'd0; par_bit <= 1'b0;
        end else begin
            tx_rd_en  <= 1'b0;
            xfer_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    uart_tx <= 1'b1; busy <= 1'b0;
                    if (!tx_empty) begin
                        sh_tx    <= tx_data;
                        par_bit  <= parity_odd;   // seed: odd=1, even=0
                        tx_rd_en <= 1'b1;
                        busy     <= 1'b1;
                        bit_cnt  <= 3'd0;
                        state    <= S_START;
                        // Ghi chú: không chờ tick ở đây
                        // → có thể trễ tối đa 1 baud period trước start bit
                        // → inter-frame gap bình thường, không ảnh hưởng receiver
                    end
                end

                S_START: if (tick) begin
                    uart_tx <= 1'b0;   // Start bit
                    state   <= S_DATA;
                end

                S_DATA: if (tick) begin
                    uart_tx <= sh_tx[0];
                    par_bit <= par_bit ^ sh_tx[0];
                    sh_tx   <= {1'b0, sh_tx[7:1]};
                    if (bit_cnt == 3'd7) begin
                        bit_cnt <= 3'd0;
                        state   <= parity_en ? S_PAR : S_STOP1;
                    end else bit_cnt <= bit_cnt + 1'b1;
                end

                S_PAR: if (tick) begin
                    uart_tx <= par_bit;
                    state   <= S_STOP1;
                end

                // [F2 FIX] xfer_done chỉ set khi không có STOP2
                // Nếu có STOP2, dời xfer_done về S_STOP2
                S_STOP1: if (tick) begin
                    uart_tx   <= 1'b1;
                    if (!stop2) begin
                        xfer_done <= 1'b1;  // Frame done ngay (1 stop bit)
                        state     <= S_IDLE;
                    end else begin
                        state     <= S_STOP2;   // Chưa done, tiếp tục stop2
                    end
                end

                // [F2 FIX] xfer_done set tại đây khi stop2=1
                S_STOP2: if (tick) begin
                    uart_tx   <= 1'b1;
                    xfer_done <= 1'b1;  // Frame hoàn toàn kết thúc
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


// =====================================================================
// UART RX Engine [F1 – nhận uart_rx đã qua 2-FF sync từ top module]
//
// Sampling strategy:
//   1. Phát hiện falling edge (start bit)
//   2. Đợi baud_div/2 clocks → sample giữa start bit
//   3. Nếu vẫn LOW → hợp lệ; đếm baud_div+1 clocks/bit
//   4. Sample 8 data bits giữa mỗi bit period
//   5. Kiểm tra stop bit = HIGH; framing error → bỏ byte
//
// uart_rx input đến đây đã được synchronize ở top module (2-FF)
// =====================================================================
module axl_uart_rx (
    input  wire        clk, rstn,
    input  wire [15:0] baud_div,
    input  wire        uart_rx,      // ← đã sync 2-FF từ top
    output reg  [7:0]  rx_data,
    output reg         rx_wr_en,
    input  wire        parity_en, parity_odd
);
    localparam S_IDLE  = 3'd0;
    localparam S_START = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_PAR   = 3'd3;
    localparam S_STOP  = 3'd4;

    reg [2:0]  state, bit_cnt;
    reg [15:0] cnt;
    reg [7:0]  sh_rx;
    reg        par_bit;
    reg        uart_rx_r;   // 1 cycle delay để detect falling edge

    wire start_edge = uart_rx_r & ~uart_rx;   // falling edge

    always @(posedge clk) begin
        if (!rstn) begin
            state     <= S_IDLE; rx_wr_en <= 1'b0;
            bit_cnt   <= 3'd0;   sh_rx    <= 8'd0;
            cnt       <= 16'd0;  par_bit  <= 1'b0;
            uart_rx_r <= 1'b1;   rx_data  <= 8'd0;
        end else begin
            rx_wr_en  <= 1'b0;
            uart_rx_r <= uart_rx;

            case (state)
                S_IDLE: begin
                    if (start_edge) begin
                        // Đợi ~baud_div/2 → giữa start bit
                        // >>1 là integer div; sai số < 0.5 clock, chấp nhận được
                        cnt   <= (baud_div >> 1);
                        state <= S_START;
                    end
                end

                S_START: begin
                    if (cnt == 16'd0) begin
                        if (!uart_rx) begin
                            // Start bit hợp lệ (vẫn LOW ở giữa period)
                            cnt     <= baud_div;
                            bit_cnt <= 3'd0;
                            par_bit <= parity_odd;
                            state   <= S_DATA;
                        end else begin
                            // Glitch / noise → bỏ qua
                            state <= S_IDLE;
                        end
                    end else cnt <= cnt - 1'b1;
                end

                S_DATA: begin
                    if (cnt == 16'd0) begin
                        sh_rx   <= {uart_rx, sh_rx[7:1]};   // LSB first
                        par_bit <= par_bit ^ uart_rx;
                        cnt     <= baud_div;
                        if (bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            state   <= parity_en ? S_PAR : S_STOP;
                        end else bit_cnt <= bit_cnt + 1'b1;
                    end else cnt <= cnt - 1'b1;
                end

                S_PAR: begin
                    if (cnt == 16'd0) begin
                        // par_bit = XOR kết quả, có thể mở rộng check error sau
                        cnt   <= baud_div;
                        state <= S_STOP;
                    end else cnt <= cnt - 1'b1;
                end

                S_STOP: begin
                    if (cnt == 16'd0) begin
                        if (uart_rx) begin
                            rx_data  <= sh_rx;
                            rx_wr_en <= 1'b1;   // Frame hợp lệ
                        end
                        // uart_rx=0 → framing error, bỏ byte (không push FIFO)
                        state <= S_IDLE;
                    end else cnt <= cnt - 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

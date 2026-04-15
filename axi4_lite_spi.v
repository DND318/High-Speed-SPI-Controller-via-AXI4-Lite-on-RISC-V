`timescale 1ns / 1ps

// =====================================================================
// MODULE: AXI4-Lite SPI Master (Thesis Grade - RISC-V Ready)
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
    
    output wire                      mosi,
    input  wire                      miso,
    output wire                      sclk,
    output wire                      ss_n,
    output wire                      irq_o
);

// Memory Map
localparam R_CTRL   = 8'h00;
localparam R_BAUD   = 8'h04;
localparam R_TXDATA = 8'h08;
localparam R_RXDATA = 8'h0C;
localparam R_STATUS = 8'h10;
localparam R_IER    = 8'h14;
localparam R_ISR    = 8'h18;

// ---------------------------------------------------------------------
// AXI4-Lite Interface Logic (Standard Handshake)
// ---------------------------------------------------------------------
reg axi_awready, axi_wready, axi_bvalid, axi_arready, axi_rvalid;
reg [31:0] axi_rdata;

assign awready = axi_awready;
assign wready  = axi_wready;
assign bresp   = 2'b00;
assign bvalid  = axi_bvalid;
assign arready = axi_arready;
assign rdata   = axi_rdata;
assign rresp   = 2'b00;
assign rvalid  = axi_rvalid;

// AWREADY & WREADY
always @(posedge aclk) begin
    if (!aresetn) begin axi_awready <= 1'b0; axi_wready <= 1'b0; end
    else begin
        if (~axi_awready && awvalid && wvalid) begin axi_awready <= 1'b1; axi_wready <= 1'b1; end
        else begin axi_awready <= 1'b0; axi_wready <= 1'b0; end
    end
end

wire slv_reg_wren = axi_wready && wvalid && axi_awready && awvalid;

// BVALID
always @(posedge aclk) begin
    if (!aresetn) axi_bvalid <= 1'b0;
    else if (slv_reg_wren && ~axi_bvalid) axi_bvalid <= 1'b1;
    else if (bready && axi_bvalid) axi_bvalid <= 1'b0;
end

// ARREADY
always @(posedge aclk) begin
    if (!aresetn) axi_arready <= 1'b0;
    else if (~axi_arready && arvalid) axi_arready <= 1'b1;
    else axi_arready <= 1'b0;
end

wire slv_reg_rden = axi_arready && arvalid && ~axi_rvalid;

// RVALID
always @(posedge aclk) begin
    if (!aresetn) axi_rvalid <= 1'b0;
    else if (slv_reg_rden) axi_rvalid <= 1'b1;
    else if (axi_rvalid && rready) axi_rvalid <= 1'b0;
end

// ---------------------------------------------------------------------
// Registers & SPI Engine Interface
// ---------------------------------------------------------------------
reg [31:0] reg_ctrl, reg_baud, reg_ier;    
reg [3:0]  reg_isr;

wire cpol = reg_ctrl[0]; wire cpha = reg_ctrl[1]; wire lsbf = reg_ctrl[2];
wire [15:0] baud_div = reg_baud[15:0];

wire baud_tick, tx_fifo_empty, tx_fifo_full, rx_fifo_empty, rx_fifo_full;
wire [31:0] tx_fifo_dout, rx_fifo_dout, rx_fifo_din;
wire tx_rd_en, rx_wr_en, busy_engine, xfer_done_engine, rx_valid_engine;

// Edge detectors for ISR
reg tx_fifo_empty_r, rx_fifo_empty_r;

// Write Registers
//synchronous write logic

reg tx_wr_en_reg;
always @(posedge aclk) begin
    if (!aresetn) begin
        reg_ctrl <= 0; reg_baud <= 32'd8; reg_ier <= 0; reg_isr <= 0; 
        tx_wr_en_reg <= 0; tx_fifo_empty_r <= 1'b1; rx_fifo_empty_r <= 1'b1;
    end else begin
        tx_wr_en_reg <= 0;
        tx_fifo_empty_r <= tx_fifo_empty;
        rx_fifo_empty_r <= rx_fifo_empty;

        // Xử lý Ghi thanh ghi (Write)
        if (slv_reg_wren) begin
            case (awaddr[7:0])
                R_CTRL:   reg_ctrl <= wdata;
                R_BAUD:   reg_baud <= wdata;
                R_TXDATA: tx_wr_en_reg <= 1'b1; // Push to FIFO
                R_IER:    reg_ier  <= wdata;
                R_ISR:    reg_isr  <= reg_isr & ~wdata[3:0]; // W1C
            endcase
        end 
        
        // Xử lý Ngắt (Phải dùng cơ chế OR để Set không bị đè bởi W1C)
        if (tx_fifo_empty && !tx_fifo_empty_r) reg_isr[0] <= 1'b1;
        if (!rx_fifo_empty && rx_fifo_empty_r) reg_isr[1] <= 1'b1;
        if (xfer_done_engine)                  reg_isr[2] <= 1'b1;
        if (tx_fifo_full)                      reg_isr[3] <= 1'b1;
    end
end

// Read Registers
wire rx_rd_en_comb = slv_reg_rden && (araddr[7:0] == R_RXDATA); // Tổ hợp Pop FIFO chuẩn FWFT

always @(posedge aclk) begin
    if (!aresetn) axi_rdata <= 0;
    else if (slv_reg_rden) begin
        case (araddr[7:0])
            R_CTRL:   axi_rdata <= reg_ctrl;
            R_BAUD:   axi_rdata <= reg_baud;
            R_RXDATA: axi_rdata <= rx_fifo_dout; 
            R_STATUS: axi_rdata <= {27'd0, rx_fifo_full, tx_fifo_empty, ~rx_fifo_empty, tx_fifo_full, busy_engine};
            R_IER:    axi_rdata <= reg_ier;
            R_ISR:    axi_rdata <= {28'd0, reg_isr};
            default:  axi_rdata <= 32'hDEADBEEF;
        endcase
    end
end

assign irq_o = |(reg_isr & reg_ier[3:0]);

// ---------------------------------------------------------------------
// Sub-modules
// ---------------------------------------------------------------------
axl_baud_div u_baud (.clk(aclk), .rstn(aresetn), .div(baud_div), .tick(baud_tick));

// FIFO TX & RX
axl_fwft_fifo u_tx_fifo (.clk(aclk), .rstn(aresetn), .wr_en(tx_wr_en_reg), .din(wdata), .rd_en(tx_rd_en), .dout(tx_fifo_dout), .empty(tx_fifo_empty), .full(tx_fifo_full));
axl_fwft_fifo u_rx_fifo (.clk(aclk), .rstn(aresetn), .wr_en(rx_wr_en), .din(rx_fifo_din), .rd_en(rx_rd_en_comb), .dout(rx_fifo_dout), .empty(rx_fifo_empty), .full(rx_fifo_full));

// SPI Engine
axl_engine u_engine (
    .clk(aclk), .rstn(aresetn), .tick(baud_tick), .cpol(cpol), .cpha(cpha), .lsbf(lsbf),
    .tx_data(tx_fifo_dout), .tx_empty(tx_fifo_empty), .tx_rd_en(tx_rd_en),
    .rx_data(rx_fifo_din), .rx_wr_en(rx_wr_en), .xfer_done(xfer_done_engine),
    .mosi(mosi), .miso(miso), .sclk(sclk), .ss_n(ss_n), .busy(busy_engine)
);
endmodule

// =====================================================================
// MODULE: Baud Divider
// =====================================================================
module axl_baud_div (input clk, rstn, input [15:0] div, output reg tick);
    reg [15:0] cnt;
    always @(posedge clk) begin
        if (!rstn) begin cnt <= 0; tick <= 0; end
        else begin
            tick <= 0;
            if (div == 0) tick <= 1;
            else if (cnt == 0) begin cnt <= div; tick <= 1; end
            else cnt <= cnt - 1;
        end
    end
endmodule

// =====================================================================
// MODULE: FWFT FIFO (First-Word Fall-Through)
// =====================================================================
module axl_fwft_fifo #(parameter WIDTH = 32, parameter DEPTH = 16)(
    input clk, rstn, wr_en, input [WIDTH-1:0] din, input rd_en, output [WIDTH-1:0] dout, output empty, full
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] wptr, rptr;
    reg [$clog2(DEPTH):0] count;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin wptr <= 0; rptr <= 0; count <= 0; end
        else begin
            if (wr_en && !full) begin mem[wptr] <= din; wptr <= wptr + 1; end
            if (rd_en && !empty) rptr <= rptr + 1;
            
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1;
                2'b01: count <= count - 1;
            endcase
        end
    end
    assign empty = (count == 0); 
    assign full  = (count == DEPTH); 
    assign dout  = mem[rptr]; // Combinational read for FWFT
endmodule

// =====================================================================
// MODULE: SPI Engine (Shift-register architecture)
// =====================================================================
module axl_engine (
    input clk, rstn, tick, cpol, cpha, lsbf,
    input [31:0] tx_data, input tx_empty, output reg tx_rd_en,
    output reg [31:0] rx_data, output reg rx_wr_en, xfer_done,
    output wire mosi, input miso, output reg sclk, ss_n, busy
);
    // Khai báo 5 trạng thái chuẩn
    localparam S_IDLE  = 3'd0;
    localparam S_SETUP = 3'd1; // Tạo t_CSS (Chip Select Setup Time)
    localparam S_PH1   = 3'd2;
    localparam S_PH2   = 3'd3;
    localparam S_HOLD  = 3'd4; // Tạo t_CSH (Chip Select Hold Time)
    
    reg [2:0] state; reg [5:0] bit_cnt;
    reg [31:0] sh_tx, sh_rx;

    assign mosi = lsbf ? sh_tx[0] : sh_tx[31];

    always @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE; ss_n <= 1; sclk <= 0; busy <= 0; tx_rd_en <= 0;
            rx_wr_en <= 0; xfer_done <= 0; bit_cnt <= 0; sh_tx <= 0; sh_rx <= 0; rx_data <= 0;
        end else begin
            tx_rd_en <= 0; rx_wr_en <= 0; xfer_done <= 0;
            
            case (state)
                S_IDLE: begin
                    ss_n <= 1; busy <= 0; sclk <= cpol;
                    if (!tx_empty) begin
                        sh_tx <= tx_data; tx_rd_en <= 1; // Pop FWFT FIFO
                        ss_n <= 0; busy <= 1; bit_cnt <= 0; 
                        state <= S_SETUP; // Chuyển sang SETUP thay vì PH1
                    end
                end
                
                // Trì hoãn 1 baud_tick trước khi bật clock (t_CSS)
                S_SETUP: if (tick) begin
                    state <= S_PH1;
                end
                
                S_PH1: if (tick) begin
                    sclk <= ~cpol;
                    if (!cpha) sh_rx <= lsbf ? {miso, sh_rx[31:1]} : {sh_rx[30:0], miso};
                    state <= S_PH2;
                end
                
                S_PH2: if (tick) begin
                    sclk <= cpol;
                    if (cpha) sh_rx <= lsbf ? {miso, sh_rx[31:1]} : {sh_rx[30:0], miso};
                    
                    if (bit_cnt == 31) begin
                        rx_data <= (cpha) ? (lsbf ? {miso, sh_rx[31:1]} : {sh_rx[30:0], miso}) : sh_rx;
                        rx_wr_en <= 1; xfer_done <= 1; 
                        state <= S_HOLD; // Chuyển sang HOLD thay vì IDLE
                    end else begin
                        bit_cnt <= bit_cnt + 1;
                        sh_tx <= lsbf ? {1'b0, sh_tx[31:1]} : {sh_tx[30:0], 1'b0};
                        state <= S_PH1;
                    end
                end
                
                // Trì hoãn 1 baud_tick sau khi tắt clock rồi mới kéo ss_n lên 1 (t_CSH)
                S_HOLD: if (tick) begin
                    ss_n <= 1;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
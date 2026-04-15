`timescale 1ns / 1ps

// =====================================================================
// MODULE: AXI4-Lite Bridge (Simple Bus → AXI4-Lite)
//
// Chức năng:
//   - Khi RISC-V vào S_MEM và địa chỉ nằm trong [PERIPH_BASE, PERIPH_END]:
//       → Khởi động giao dịch AXI4-Lite tương ứng (Write hoặc Read)
//       → Sau khi handshake hoàn tất, assert mem_ready = 1 (1 cycle)
//       → RISC-V thoát khỏi S_MEM, tiếp tục thực thi
//   - Khi địa chỉ là SRAM (không phải peripheral):
//       → Truy cập SRAM trực tiếp, assert mem_ready ngay lập tức
//
// Timing Diagram (AXI4-Lite Write):
//  Cycle: │  1  │  2  │  3  │  4  │  5  │
//  State: │IDLE │ WR_A│ WR_R│DONE │IDLE │
//  CPU:   │S_MEM│S_MEM│S_MEM│S_MEM│S_IF │
//  awvld: │  0  │  1  │  0  │  0  │  0  │
//  wvld:  │  0  │  1  │  0  │  0  │  0  │
//  bvld:  │  0  │  0  │  1  │  0  │  0  │  ← peripheral ack
//  mrdy:  │  0  │  0  │  0  │  1  │  0  │  ← CPU tiếp tục
//
// Timing Diagram (AXI4-Lite Read):
//  Cycle: │  1  │  2  │  3  │DONE │
//  State: │IDLE │ RD_A│ RD_D│     │
//  CPU:   │S_MEM│S_MEM│S_MEM│S_WB │
//  arvld: │  0  │  1  │  0  │  0  │
//  rvld:  │  0  │  0  │  1  │  0  │  ← rdata valid
//  mrdy:  │  0  │  0  │  1  │  0  │  ← CPU tiếp tục
//
// =====================================================================
module axi4_lite_bridge #(
    // Vùng địa chỉ peripheral (SPI): [PERIPH_BASE, PERIPH_BASE + PERIPH_SIZE)
    parameter [31:0] PERIPH_BASE = 32'h0001_0000,
    parameter [31:0] PERIPH_SIZE = 32'h0000_0020  // 32 bytes = 8 registers × 4
)(
    input  wire        clk,
    input  wire        reset,

    // ---- Phía CPU (Simple Bus) ----
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    input  wire        cpu_wen,      // 1 = ghi, 0 = đọc (chỉ có nghĩa khi cpu_req=1)
    input  wire        cpu_req,      // Yêu cầu truy cập (= dmem_wen hoặc state==S_MEM)
    output reg  [31:0] cpu_rdata,
    output reg         mem_ready,    // Pulse 1 cycle: CPU có thể thoát S_MEM

    // ---- Phía AXI4-Lite Master (kết nối đến SPI Controller) ----
    output reg  [7:0]  m_awaddr,
    output reg         m_awvalid,
    input  wire        m_awready,
    output reg  [31:0] m_wdata,
    output reg  [3:0]  m_wstrb,
    output reg         m_wvalid,
    input  wire        m_wready,
    input  wire [1:0]  m_bresp,
    input  wire        m_bvalid,
    output reg         m_bready,
    output reg  [7:0]  m_araddr,
    output reg         m_arvalid,
    input  wire        m_arready,
    input  wire [31:0] m_rdata,
    input  wire [1:0]  m_rresp,
    input  wire        m_rvalid,
    output reg         m_rready,

    // ---- Phía SRAM (kết nối đến SRAM trực tiếp) ----
    output wire [31:0] sram_addr,
    output wire [31:0] sram_wdata,
    output wire        sram_wen,
    input  wire [31:0] sram_rdata
);

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    localparam [2:0]
        BR_IDLE   = 3'd0,
        BR_WR_A   = 3'd1,   // AXI Write: gửi địa chỉ + dữ liệu
        BR_WR_R   = 3'd2,   // AXI Write: chờ BRESP
        BR_RD_A   = 3'd3,   // AXI Read: gửi địa chỉ
        BR_RD_D   = 3'd4,   // AXI Read: chờ RDATA
        BR_DONE   = 3'd5;   // Hoàn thành, pulse mem_ready

    reg [2:0] state;

    // Kiểm tra địa chỉ có thuộc về peripheral không
    wire is_periph = (cpu_addr >= PERIPH_BASE) &&
                     (cpu_addr < (PERIPH_BASE + PERIPH_SIZE));
    wire [7:0] periph_offset = cpu_addr[7:0]; // Offset trong không gian SPI

    // SRAM passthrough: luôn kết nối, nhưng wen chỉ active khi không phải periph
    assign sram_addr  = cpu_addr;
    assign sram_wdata = cpu_wdata;
    assign sram_wen   = cpu_wen & ~is_periph;

    // ----------------------------------------------------------------
    // FSM chính
    // ----------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state      <= BR_IDLE;
            mem_ready  <= 1'b0;
            cpu_rdata  <= 32'd0;
            m_awvalid  <= 1'b0;  m_awaddr <= 8'd0;
            m_wvalid   <= 1'b0;  m_wdata  <= 32'd0;  m_wstrb  <= 4'b1111;
            m_bready   <= 1'b0;
            m_arvalid  <= 1'b0;  m_araddr <= 8'd0;
            m_rready   <= 1'b0;
        end else begin
            // Mặc định: tắt mem_ready sau 1 chu kỳ pulse
            mem_ready <= 1'b0;

            case (state)

                // ---- Chờ yêu cầu từ CPU ----
                BR_IDLE: begin
                    if (cpu_req) begin
                        if (!is_periph) begin
                            // SRAM: ghi được thực hiện tổ hợp, đọc 1 cycle
                            cpu_rdata <= sram_rdata;
                            mem_ready <= 1'b1;       // Sẵn sàng ngay
                            // state không đổi, IDLE cycle tiếp theo
                        end else begin
                            // Peripheral AXI4-Lite
                            if (cpu_wen) begin
                                // Write: gửi awaddr + wdata đồng thời
                                m_awaddr  <= periph_offset;
                                m_awvalid <= 1'b1;
                                m_wdata   <= cpu_wdata;
                                m_wstrb   <= 4'b1111;
                                m_wvalid  <= 1'b1;
                                state     <= BR_WR_A;
                            end else begin
                                // Read: gửi araddr
                                m_araddr  <= periph_offset;
                                m_arvalid <= 1'b1;
                                state     <= BR_RD_A;
                            end
                        end
                    end
                end

                // ---- AXI Write: chờ awready & wready ----
                // AXI4-Lite cho phép awready và wready đến cùng lúc hoặc lần lượt
                BR_WR_A: begin
                    if (m_awready) m_awvalid <= 1'b0;  // awaddr đã được chấp nhận
                    if (m_wready)  m_wvalid  <= 1'b0;  // wdata đã được chấp nhận

                    // Khi cả hai kênh đều được chấp nhận → chờ BRESP
                    if ((m_awready || !m_awvalid) && (m_wready || !m_wvalid)) begin
                        m_bready <= 1'b1;
                        state    <= BR_WR_R;
                    end
                end

                // ---- AXI Write: chờ BVALID (write response) ----
                BR_WR_R: begin
                    if (m_bvalid && m_bready) begin
                        m_bready  <= 1'b0;
                        mem_ready <= 1'b1;   // Giao dịch ghi hoàn thành
                        state     <= BR_IDLE;
                    end
                end

                // ---- AXI Read: chờ arready ----
                BR_RD_A: begin
                    if (m_arready) begin
                        m_arvalid <= 1'b0;
                        m_rready  <= 1'b1;
                        state     <= BR_RD_D;
                    end
                end

                // ---- AXI Read: chờ RVALID (read data) ----
                BR_RD_D: begin
                    if (m_rvalid && m_rready) begin
                        cpu_rdata <= m_rdata;   // Lấy dữ liệu trả về cho CPU
                        m_rready  <= 1'b0;
                        mem_ready <= 1'b1;      // Giao dịch đọc hoàn thành
                        state     <= BR_IDLE;
                    end
                end

                default: state <= BR_IDLE;
            endcase
        end
    end

endmodule
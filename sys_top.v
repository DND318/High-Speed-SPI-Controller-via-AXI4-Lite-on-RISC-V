`timescale 1ns / 1ps

// MODULE: sys_top (Hệ thống SoC RISC-V RV32IMZicsr tích hợp SPI/UART)
module sys_top (
    input  wire        clk,
    input  wire        reset,      
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_sclk,
    output wire        spi_ss_n,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        trap_o,
    output wire [31:0] mcause_o,
    output wire [31:0] mepc_o,
    output wire [2:0]  cpu_state_dbg
);
    wire [31:0] cpu_addr;
    wire [31:0] cpu_wdata;
    wire [3:0]  cpu_wstrb;
    wire [31:0] cpu_rdata;
    wire        mem_ready;
    wire [31:0] cpu_pc;
    wire [31:0] cpu_alu_out;


    // CPU yêu cầu truy cập bộ nhớ khi FSM ở trạng thái S_MEM (3'd3)
    wire cpu_req = (cpu_state_dbg == 3'd3);
    wire cpu_wen = |cpu_wstrb;

    // 2. Tín hiệu trung gian AXI4-Lite (Bridge Master -> SPI/UART Slave)
    wire [7:0]  axl_awaddr;
    wire        axl_awvalid;
    wire        axl_awready;
    wire [31:0] axl_wdata;
    wire [3:0]  axl_wstrb;
    wire        axl_wvalid;
    wire        axl_wready;
    wire [1:0]  axl_bresp;
    wire        axl_bvalid;
    wire        axl_bready;
    wire [7:0]  axl_araddr;
    wire        axl_arvalid;
    wire        axl_arready;
    wire [31:0] axl_rdata;
    wire [1:0]  axl_rresp;
    wire        axl_rvalid;
    wire        axl_rready;
    wire [31:0] sram_addr;
    wire [31:0] sram_wdata;
    wire        sram_wen;
    wire [31:0] sram_rdata;

    // Tạo tín hiệu write-enable cho từng byte trong RAM để hỗ trợ sb, sh, sw
    wire [3:0]  sram_byte_en = sram_wen ? cpu_wstrb : 4'b0000;
    wire aresetn = ~reset;
    Datapath_Multi_cycle_Processor_RISC_V u_cpu (
        .clk(clk),
        .reset(reset),
        .PC_out(cpu_pc),
        .ALUResult_out(cpu_alu_out),
        .state_dbg(cpu_state_dbg),
        .dmem_addr(cpu_addr),
        .dmem_wdata(cpu_wdata),
        .dmem_wstrb(cpu_wstrb),
        .dmem_rdata(cpu_rdata),
        .mem_ready(mem_ready),
        .trap_o(trap_o),
        .mcause_o(mcause_o),
        .mepc_o(mepc_o)
    );
    axi4_lite_bridge #(
        .PERIPH_BASE(32'h0001_0000), // Địa chỉ cơ sở của khối SPI/UART ngoại vi
        .PERIPH_SIZE(32'h0000_0020) 
    ) u_bridge (
        .clk(clk),
        .reset(reset),

        // CPU Interface
        .cpu_addr(cpu_addr),
        .cpu_wdata(cpu_wdata),
        .cpu_wen(cpu_wen),
        .cpu_req(cpu_req),
        .cpu_rdata(cpu_rdata),
        .mem_ready(mem_ready),

        // AXI4-Lite Master Interface
        .m_awaddr(axl_awaddr),
        .m_awvalid(axl_awvalid),
        .m_awready(axl_awready),
        .m_wdata(axl_wdata),
        .m_wstrb(axl_wstrb),
        .m_wvalid(axl_wvalid),
        .m_wready(axl_wready),
        .m_bresp(axl_bresp),
        .m_bvalid(axl_bvalid),
        .m_bready(axl_bready),
        .m_araddr(axl_araddr),
        .m_arvalid(axl_arvalid),
        .m_arready(axl_arready),
        .m_rdata(axl_rdata),
        .m_rresp(axl_rresp),
        .m_rvalid(axl_rvalid),
        .m_rready(axl_rready),

        // SRAM Interface
        .sram_addr(sram_addr),
        .sram_wdata(sram_wdata),
        .sram_wen(sram_wen),
        .sram_rdata(sram_rdata)
    );


    // Instance 3: AXI4-Lite SPI/UART Dual-Mode Controller
    axi4_lite_spi #(
        .AXI_ADDR_WIDTH(8)
    ) u_periph (
        .aclk(clk),
        .aresetn(aresetn),

        // AXI4-Lite Slave Interface
        .awaddr(axl_awaddr),
        .awvalid(axl_awvalid),
        .awready(axl_awready),
        .wdata(axl_wdata),
        .wstrb(axl_wstrb),
        .wvalid(axl_wvalid),
        .wready(axl_wready),
        .bresp(axl_bresp),
        .bvalid(axl_bvalid),
        .bready(axl_bready),
        .araddr(axl_araddr),
        .arvalid(axl_arvalid),
        .arready(axl_arready),
        .rdata(axl_rdata),
        .rresp(axl_rresp),
        .rvalid(axl_rvalid),
        .rready(axl_rready),

        // Pins ngoại vi
        .mosi(spi_mosi),
        .miso(spi_miso),
        .sclk(spi_sclk),
        .ss_n(spi_ss_n),
        .uart_tx(uart_tx),
        .uart_rx(uart_rx),
        .irq_o() // Bỏ trống nếu không sử dụng ngắt cứng lên CPU
    );

    // ─────────────────────────────────────────────────────────────────
    // Instance 4: Khối RAM nội bộ (Byte-Enabled SRAM - 4KB)
    // ─────────────────────────────────────────────────────────────────
    sram_byte_enabled #(
        .WORDS(1024) // 1024 words * 4 bytes = 4096 Bytes
    ) u_sram (
        .clk(clk),
        .addr(sram_addr[11:2]), // Chia 4 (bỏ 2 bit LSB) để căn chỉnh word 32-bit
        .din(sram_wdata),
        .be(sram_byte_en),
        .we(sram_wen),
        .dout(sram_rdata)
    );

endmodule


// =====================================================================
// MODULE PHỤ: Bộ nhớ RAM hỗ trợ ghi theo từng byte đơn lẻ (Đã sửa lỗi)
// =====================================================================
module sram_byte_enabled #(
    parameter WORDS = 1024
)(
    input  wire        clk,
    input  wire [9:0]  addr,
    input  wire [31:0] din,
    input  wire [3:0]  be,    // Byte enable mask
    input  wire        we,    // Write enable tổng
    output wire [31:0] dout   // Chuyển sang wire để đọc tổ hợp
);
    reg [31:0] mem [0:WORDS-1];

    // Khởi tạo bộ nhớ tránh lỗi X-propagation
    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1) begin
            mem[i] = 32'd0;
        end
    end

    // Ghi đồng bộ theo Byte Enable
    always @(posedge clk) begin
        if (we) begin
            if (be[0]) mem[addr][7:0]   <= din[7:0];
            if (be[1]) mem[addr][15:8]  <= din[15:8];
            if (be[2]) mem[addr][23:16] <= din[23:16];
            if (be[3]) mem[addr][31:24] <= din[31:24];
        end
    end

    // Đọc tổ hợp trực tiếp nhằm triệt tiêu Race Condition
    assign dout = mem[addr];

endmodule
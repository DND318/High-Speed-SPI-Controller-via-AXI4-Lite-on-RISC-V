`timescale 1ns / 1ps

// MODULE: sys_top_tb 
module sys_top_tb;

    // 1. Tín hiệu kết nối
    reg         clk;
    reg         reset;

    wire        spi_mosi;
    wire        spi_miso;
    wire        spi_sclk;
    wire        spi_ss_n;

    wire        uart_tx;
    wire        uart_rx;

    wire        trap_o;
    wire [31:0] mcause_o;
    wire [31:0] mepc_o;
    wire [2:0]  cpu_state_dbg;

    // 2.(Loopback)
    assign spi_miso = spi_mosi;
    assign uart_rx  = uart_tx;

    // 3. Khởi tạo SoC (UUT)
    sys_top uut (
        .clk(clk),
        .reset(reset),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_sclk(spi_sclk),
        .spi_ss_n(spi_ss_n),
        .uart_tx(uart_tx),
        .uart_rx(uart_rx),
        .trap_o(trap_o),
        .mcause_o(mcause_o),
        .mepc_o(mepc_o),
        .cpu_state_dbg(cpu_state_dbg)
    );

    // 4. Tạo clock 50MHz (Chu kỳ 20ns)
    always begin
        #10 clk = ~clk;
    end

    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars(0, sys_top_tb);
        clk   = 0;
        reset = 1;

        $display("=================================================================");
        $display("   BAT DAU MO PHONG TREN SOC SYSTEM");
        $display("=================================================================");
        
        #100;
        reset = 0;
        $display("[TIME: %0t ns] Reset hoan thanh. CPU bat dau chay...", $time);

        // Thời gian mô phỏng 80us
        #80000; //spi
		//#150000; //uart

        $display("=================================================================");
        $display("   KET THUC MO PHONG");
        $display("   Gia tri cuoi cung tai SRAM[0]: 32'h%h", uut.u_sram.mem[0]);
        $display("=================================================================");
        $finish; 
    end

    // 6. Giám sát RAM đồng bộ theo Clock 
    always @(posedge clk) begin
        if (uut.u_sram.we && (uut.u_sram.addr == 10'd0)) begin
            #1; 
            $display("[TIME: %0t ns] >> KET QUA: SRAM[0] da nhan duoc: 32'h%h (%0d)", 
                     $time, uut.u_sram.mem[0], uut.u_sram.mem[0]);
        end
    end
    integer cycle_cnt = 0;
    reg x_detected = 0;

    always @(posedge clk) begin
        if (!reset) begin
            cycle_cnt = cycle_cnt + 1;
            
            // Chỉ in 80 chu kỳ đầu tiên HOẶC in khi bắt đầu phát hiện PC bị lỗi X
            if (cycle_cnt < 80 || (uut.cpu_pc === 32'dx && !x_detected)) begin
                if (uut.cpu_pc === 32'dx) begin
                    x_detected = 1; // Chỉ in cảnh báo lỗi X một lần duy nhất để tránh tràn log
                    $display("\n!!! PHÁT HIỆN LỖI: PC BI CHUYEN THANH X TAI CHU KY NAY !!!");
                end
                
                $display("[CLOCK %0d | TIME: %0t ps]", cycle_cnt, $time);
                $display("  -> [CPU]    PC: 32'h%h | State: %0d | cpu_req: %b | mem_ready: %b | ALUOut: 32'h%h", 
                         uut.cpu_pc, uut.cpu_state_dbg, uut.cpu_req, uut.mem_ready, uut.cpu_alu_out);
                $display("  -> [BRIDGE] State: %0d | sram_wen: %b | is_periph: %b | cpu_rdata: 32'h%h", 
                         uut.u_bridge.state, uut.u_bridge.sram_wen, uut.u_bridge.is_periph, uut.cpu_rdata);
                $display("  -> [SPI]    Ctrl: 32'h%h | Baud: 32'h%h | TX_Empty: %b | RX_Empty: %b | busy_engine: %b", 
                         uut.u_periph.reg_ctrl, uut.u_periph.reg_baud, uut.u_periph.tx_fifo_empty, uut.u_periph.rx_fifo_empty, uut.u_periph.busy_engine);
                $display("  -> [ENGINE] SPI State: %0d | sclk: %b | mosi: %b | miso: %b",
                         uut.u_periph.u_spi_engine.state, uut.spi_sclk, uut.spi_mosi, uut.spi_miso);
                $display("-----------------------------------------------------------------");
            end
        end
    end
endmodule

`timescale 1ns/1ps

module tb_spi_uart_fair;


  reg [31:0] DATA     = 32'h12345678;  
  reg [15:0] SPI_DIV  = 16'd1;          // SPI : 2*(SPI_DIV+1)  
  reg [15:0] UART_DIV = 16'd3;          // UART: (UART_DIV+1)   cl


  reg         aclk = 0, aresetn;
  reg  [7:0]  awaddr, araddr;
  reg         awvalid, wvalid, bready, arvalid, rready;
  reg  [31:0] wdata;  reg [3:0] wstrb;
  wire        awready, wready, bvalid, arready, rvalid;
  wire [1:0]  bresp, rresp;  wire [31:0] rdata;
  wire        mosi, sclk, ss_n, uart_tx, irq;
  reg         miso_r = 0;


  always @(posedge aclk) miso_r <= mosi;   
  wire uart_rx = uart_tx;                   

  localparam R_CTRL=8'h00, R_BAUD=8'h04, R_TXDATA=8'h08, R_RXDATA=8'h0C, R_STATUS=8'h10;

  axi4_lite_spi #(.AXI_ADDR_WIDTH(8)) dut (
    .aclk(aclk), .aresetn(aresetn),
    .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready),
    .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rresp(rresp), .rvalid(rvalid), .rready(rready),
    .mosi(mosi), .miso(miso_r), .sclk(sclk), .ss_n(ss_n),
    .uart_tx(uart_tx), .uart_rx(uart_rx), .irq_o(irq)
  );

  always #5 aclk = ~aclk;  
  integer cyc = 0;
  always @(posedge aclk) cyc = cyc + 1;


  integer done_cnt = 0;
  always @(posedge aclk) if (dut.xfer_done_engine) done_cnt = done_cnt + 1;

  integer pass=0, fail=0;

  // ---- AXI write task ----
  task axi_write(input [7:0] addr, input [31:0] data);
    begin
      @(posedge aclk);
      awaddr<=addr; awvalid<=1; wdata<=data; wstrb<=4'hF; wvalid<=1; bready<=1;
      wait(awready && wready); @(posedge aclk);
      awvalid<=0; wvalid<=0; wait(bvalid); @(posedge aclk); bready<=0;
    end
  endtask

  // ---- AXI read task ----
  task axi_read(input [7:0] addr, output [31:0] data);
    begin
      @(posedge aclk);
      araddr<=addr; arvalid<=1; rready<=1;
      wait(arready); @(posedge aclk); arvalid<=0;
      wait(rvalid); data=rdata; @(posedge aclk); rready<=0;
    end
  endtask

  integer t0, t_spi, t_uart;
  reg [31:0] spi_rx, tmp;
  reg [7:0]  ub [0:3];
  reg [31:0] uart_rx_word;
  integer i;

  initial begin
    $dumpfile("dump.vcd"); $dumpvars(0, tb_spi_uart_fair);
    awvalid=0; wvalid=0; bready=0; arvalid=0; rready=0;
    awaddr=0; araddr=0; wdata=0; wstrb=0;
    aresetn=0; repeat(10) @(posedge aclk); aresetn=1;
    repeat(5) @(posedge aclk);

    $display("================================================================");
    $display(" SO SANH CONG BANG: 32 BIT qua SPI vs UART");
    $display(" DATA = 0x%h | SPI_DIV = %0d | UART_DIV = %0d | f_clk = 100 MHz",
             DATA, SPI_DIV, UART_DIV);
    $display(" SPI : %0d clock/bit  |  UART: %0d clock/bit",
             2*(SPI_DIV+1), (UART_DIV+1));
    $display("================================================================");

    // ---------------- SPI: 1 word 32-bit ----------------
    axi_write(R_CTRL, 32'h0);                  // MODE = SPI, mode 0
    axi_write(R_BAUD, {16'd0, SPI_DIV});       // <-- SPI_DIV
    repeat(5) @(posedge aclk);
    fork
      begin @(negedge ss_n); t0=cyc; @(posedge ss_n); t_spi=cyc-t0; end
      axi_write(R_TXDATA, DATA);
    join
    repeat(5) @(posedge aclk);
    axi_read(R_RXDATA, spi_rx);
    $display("[SPI ] gui 0x%h -> nhan 0x%h : %s", DATA, spi_rx,
             (spi_rx===DATA)?"LOOPBACK OK":"SAI");
    if (spi_rx===DATA) pass=pass+1; else fail=fail+1;
    $display("[SPI ] 32 bit het %0d cycles (%0d ns)", t_spi, t_spi*10);

    repeat(20) @(posedge aclk);

    // ---------------- UART: 4 byte lien tiep (32 bit) ----------------
    axi_write(R_CTRL, 32'h8);                  // MODE = UART, 8N1
    axi_write(R_BAUD, {16'd0, UART_DIV});      // <-- UART_DIV
    repeat(5) @(posedge aclk);
    ub[0]=DATA[7:0]; ub[1]=DATA[15:8]; ub[2]=DATA[23:16]; ub[3]=DATA[31:24];
    done_cnt = 0;
    fork
      begin : MEAS
        @(negedge uart_tx); t0 = cyc;          // start bit byte dau
        wait(done_cnt == 4);                   // het 4 frame
        @(posedge aclk); t_uart = cyc - t0;
      end
      begin : FILL
        axi_write(R_TXDATA, {24'd0, ub[0]});   // ghi 4 byte LIEN TIEP vao FIFO
        axi_write(R_TXDATA, {24'd0, ub[1]});
        axi_write(R_TXDATA, {24'd0, ub[2]});
        axi_write(R_TXDATA, {24'd0, ub[3]});
      end
    join
    for (i=0;i<4;i=i+1) begin axi_read(R_RXDATA, tmp); ub[i]=tmp[7:0]; end
    uart_rx_word = {ub[3],ub[2],ub[1],ub[0]};
    $display("[UART] gui 0x%h -> nhan 0x%h : %s", DATA, uart_rx_word,
             (uart_rx_word===DATA)?"LOOPBACK OK":"SAI");
    if (uart_rx_word===DATA) pass=pass+1; else fail=fail+1;
    $display("[UART] 32 bit (4 frame) het %0d cycles (%0d ns)", t_uart, t_uart*10);

    // ---------------- So sanh ----------------
    $display("================================================================");
    $display(" KET QUA (cung 32 bit du lieu):");
    $display("----------------------------------------------------------------");
    $display("   SPI  = %0d cycles (%0d ns)  -- 32 bit, 0 overhead", t_spi, t_spi*10);
    $display("   UART = %0d cycles (%0d ns)  -- 40 bit (8 overhead start/stop)", t_uart, t_uart*10);
    $display("----------------------------------------------------------------");
    if (t_spi < t_uart)
      $display(" => SPI NHANH HON %0d cycles (%0d ns), nhanh hon %0d%%",
               t_uart-t_spi, (t_uart-t_spi)*10, ((t_uart-t_spi)*100)/t_uart);
    else if (t_uart < t_spi)
      $display(" => UART NHANH HON %0d cycles (%0d ns), nhanh hon %0d%%",
               t_spi-t_uart, (t_spi-t_uart)*10, ((t_spi-t_uart)*100)/t_spi);
    else
      $display(" => BANG NHAU");
    $display("----------------------------------------------------------------");
    $display(" Throughput du lieu (32 data bit / thoi gian):");
    $display("   SPI  = %0d Mbps", (32*1000)/(t_spi*10));
    $display("   UART = %0d Mbps", (32*1000)/(t_uart*10));
    $display("================================================================");
    $display(" PASS=%0d FAIL=%0d", pass, fail);
    $display("================================================================");
    $finish;
  end

  initial begin #20000000; $display("TIMEOUT!"); $finish; end
endmodule

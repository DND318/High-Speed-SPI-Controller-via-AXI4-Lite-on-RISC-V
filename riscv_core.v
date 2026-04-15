`timescale 1ns / 1ps

// =====================================================================
// ALU Operation Codes (tương thích với single-cycle version)
// =====================================================================
`define ALU_NONE                          5'b00000
`define ALU_SHIFTL                        5'b00001
`define ALU_SHIFTR                        5'b00010
`define ALU_SHIFTR_ARITH                  5'b00011
`define ALU_ADD                           5'b00100
`define ALU_SUB                           5'b00110
`define ALU_AND                           5'b00111
`define ALU_OR                            5'b01000
`define ALU_XOR                           5'b01001
`define ALU_LESS_THAN                     5'b01010
`define ALU_LESS_THAN_SIGNED              5'b01011
`define ALU_GREATER_THAN_OR_EQUAL         5'b01100
`define ALU_GREATER_THAN_OR_EQUAL_SIGNED  5'b01101
`define ALU_EQUAL                         5'b01110
`define ALU_NOT_EQUAL                     5'b01111
`define ALU_LOAD_UPPER                    5'b10000

// FSM States
`define S_IF   3'd0   // Instruction Fetch
`define S_ID   3'd1   // Instruction Decode / Register Read
`define S_EX   3'd2   // Execute
`define S_MEM  3'd3   // Memory Access
`define S_WB   3'd4   // Write Back

// =====================================================================
// MODULE: Multi-Cycle RISC-V Processor (RV32I Subset)
// Kiến trúc Harvard (I-Mem & D-Mem riêng biệt)
// Giao tiếp D-Mem tương thích với AXI4-Lite SPI Master
// =====================================================================
//
//  Luồng thực thi theo opcode:
//  ┌─────────────────────────────────────────────────────────┐
//  │  R-type / I-type / LUI / AUIPC : IF→ID→EX→WB           │
//  │  Load                          : IF→ID→EX→MEM→WB        │
//  │  Store                         : IF→ID→EX→MEM→IF        │
//  │  Branch (taken / not-taken)    : IF→ID→EX→IF            │
//  │  JAL / JALR                    : IF→ID→EX→WB            │
//  └─────────────────────────────────────────────────────────┘
//
module Datapath_Multi_cycle_Processor_RISC_V (
    input        clk,
    input        reset,
    // Debug / Interface outputs
    output [31:0] PC_out,
    output [31:0] ALUResult_out,
    output [31:0] dmem_addr,
    output [31:0] dmem_wdata,
    input  [31:0] dmem_rdata,
    output        dmem_wen,
    // Stall: giữ ở S_MEM đến khi mem_ready=1 (dùng cho AXI4-Lite bridge)
    input         mem_ready,
    output [2:0]  state_dbg        // Xuất trạng thái FSM để debug
);

    // ----------------------------------------------------------------
    // Thanh ghi pipeline (Multi-cycle Intermediate Registers)
    // ----------------------------------------------------------------
    reg [31:0] PC;           // Program Counter
    reg [31:0] IR;           // Instruction Register
    reg [31:0] A;            // Thanh ghi đầu ra RS1 của Register File
    reg [31:0] B;            // Thanh ghi đầu ra RS2 của Register File
    reg [31:0] ALUOut;       // Kết quả ALU đã chốt
    reg [31:0] MDR;          // Memory Data Register (kết quả đọc bộ nhớ)
    reg [31:0] PCPlus4_reg;  // PC+4 đã lưu (dùng cho WB của JAL/JALR)
    reg [2:0]  state;        // Trạng thái FSM hiện tại

    assign PC_out        = PC;
    assign ALUResult_out = ALUOut;
    assign state_dbg     = state;

    // ----------------------------------------------------------------
    // Giải mã instruction fields từ IR
    // ----------------------------------------------------------------
    wire [6:0] opcode = IR[6:0];
    wire [2:0] funct3 = IR[14:12];
    wire [6:0] funct7 = IR[31:25];
    wire [4:0] rs1    = IR[19:15];
    wire [4:0] rs2    = IR[24:20];
    wire [4:0] rd     = IR[11:7];

    // ----------------------------------------------------------------
    // Tín hiệu điều khiển tổ hợp (sinh từ opcode + funct3/7)
    // Luôn active – FSM chỉ dùng để khoá thời điểm ghi
    // ----------------------------------------------------------------
    reg [2:0]  ImmSrc;
    reg [4:0]  ALUCtrl;
    reg        ALUSrc;       // 0: SrcB = B (register), 1: SrcB = ImmExt
    reg        UIPC_add;     // 0: SrcA = A (register), 1: SrcA = PC
    reg [1:0]  ResultSrc;    // 00: ALUOut, 01: MDR, 10: PCPlus4_reg
    reg        RegWrite_en;
    reg        MemWrite_en;
    reg        is_branch;
    reg        is_jalr;

    always @(*) begin
        // Giá trị mặc định an toàn
        ImmSrc      = 3'b000;
        ALUCtrl     = `ALU_ADD;
        ALUSrc      = 1'b0;
        UIPC_add    = 1'b0;
        ResultSrc   = 2'b00;
        RegWrite_en = 1'b0;
        MemWrite_en = 1'b0;
        is_branch   = 1'b0;
        is_jalr     = 1'b0;

        case (opcode)
            // ---- R-type (ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU) ----
            7'b0110011: begin
                RegWrite_en = 1'b1;
                case ({funct7[5], funct3})
                    4'b0_000: ALUCtrl = `ALU_ADD;
                    4'b1_000: ALUCtrl = `ALU_SUB;
                    4'b0_001: ALUCtrl = `ALU_SHIFTL;
                    4'b0_010: ALUCtrl = `ALU_LESS_THAN_SIGNED;
                    4'b0_011: ALUCtrl = `ALU_LESS_THAN;
                    4'b0_100: ALUCtrl = `ALU_XOR;
                    4'b0_101: ALUCtrl = `ALU_SHIFTR;
                    4'b1_101: ALUCtrl = `ALU_SHIFTR_ARITH;
                    4'b0_110: ALUCtrl = `ALU_OR;
                    4'b0_111: ALUCtrl = `ALU_AND;
                    default:  ALUCtrl = `ALU_ADD;
                endcase
            end

            // ---- I-type ALU (ADDI, ANDI, ORI, XORI, SLTI, SLTIU, SLLI, SRLI, SRAI) ----
            7'b0010011: begin
                RegWrite_en = 1'b1;
                ALUSrc      = 1'b1;
                ImmSrc      = 3'b000;
                case (funct3)
                    3'b000: ALUCtrl = `ALU_ADD;
                    3'b001: ALUCtrl = `ALU_SHIFTL;
                    3'b010: ALUCtrl = `ALU_LESS_THAN_SIGNED;
                    3'b011: ALUCtrl = `ALU_LESS_THAN;
                    3'b100: ALUCtrl = `ALU_XOR;
                    3'b101: ALUCtrl = funct7[5] ? `ALU_SHIFTR_ARITH : `ALU_SHIFTR;
                    3'b110: ALUCtrl = `ALU_OR;
                    3'b111: ALUCtrl = `ALU_AND;
                    default: ALUCtrl = `ALU_ADD;
                endcase
            end

            // ---- Load (LW, LH, LB, LHU, LBU) ----
            7'b0000011: begin
                RegWrite_en = 1'b1;
                ALUSrc      = 1'b1;
                ImmSrc      = 3'b000;
                ALUCtrl     = `ALU_ADD;
                ResultSrc   = 2'b01;   // Kết quả WB = MDR
            end

            // ---- Store (SW, SH, SB) ----
            7'b0100011: begin
                ImmSrc      = 3'b001;
                ALUSrc      = 1'b1;
                ALUCtrl     = `ALU_ADD;
                MemWrite_en = 1'b1;
            end

            // ---- Branch (BEQ, BNE, BLT, BGE, BLTU, BGEU) ----
            7'b1100011: begin
                ImmSrc    = 3'b010;
                is_branch = 1'b1;
                // SrcA = A (rs1), SrcB = B (rs2) → so sánh hai thanh ghi
                case (funct3)
                    3'b000: ALUCtrl = `ALU_EQUAL;
                    3'b001: ALUCtrl = `ALU_NOT_EQUAL;
                    3'b100: ALUCtrl = `ALU_LESS_THAN_SIGNED;
                    3'b101: ALUCtrl = `ALU_GREATER_THAN_OR_EQUAL_SIGNED;
                    3'b110: ALUCtrl = `ALU_LESS_THAN;
                    3'b111: ALUCtrl = `ALU_GREATER_THAN_OR_EQUAL;
                    default: ALUCtrl = `ALU_EQUAL;
                endcase
            end

            // ---- JAL (Jump And Link) ----
            // ALU tính địa chỉ nhảy: PC + ImmExt(J-type)
            7'b1101111: begin
                RegWrite_en = 1'b1;
                ImmSrc      = 3'b011;   // J-type
                UIPC_add    = 1'b1;     // SrcA = PC
                ALUSrc      = 1'b1;
                ALUCtrl     = `ALU_ADD;
                ResultSrc   = 2'b10;    // Ghi PC+4 vào rd (return address)
            end

            // ---- JALR (Jump And Link Register) ----
            // ALU tính: rs1 + ImmExt, sau đó clear bit[0]
            7'b1100111: begin
                RegWrite_en = 1'b1;
                ImmSrc      = 3'b000;
                ALUSrc      = 1'b1;
                ALUCtrl     = `ALU_ADD;
                is_jalr     = 1'b1;
                ResultSrc   = 2'b10;    // Ghi PC+4 vào rd
            end

            // ---- LUI (Load Upper Immediate) ----
            7'b0110111: begin
                RegWrite_en = 1'b1;
                ImmSrc      = 3'b100;   // U-type
                ALUSrc      = 1'b1;
                ALUCtrl     = `ALU_LOAD_UPPER;
            end

            // ---- AUIPC (Add Upper Immediate to PC) ----
            7'b0010111: begin
                RegWrite_en = 1'b1;
                ImmSrc      = 3'b100;
                UIPC_add    = 1'b1;
                ALUSrc      = 1'b1;
                ALUCtrl     = `ALU_ADD;
            end

            default: begin /* NOP / Undefined – tín hiệu default an toàn */ end
        endcase
    end

    // ----------------------------------------------------------------
    // Mở rộng Immediate
    // ----------------------------------------------------------------
    wire [31:0] ImmExt;
    extend ext_inst (
        .ImmSrc (ImmSrc),
        .Imm    (IR[31:7]),
        .ImmExt (ImmExt)
    );

    // ----------------------------------------------------------------
    // ALU Sources
    // ----------------------------------------------------------------
    wire [31:0] SrcA = UIPC_add ? PC : A;
    wire [31:0] SrcB = ALUSrc   ? ImmExt : B;

    wire [31:0] ALU_out_wire;
    alu alu_inst (
        .alu_op_i (ALUCtrl),
        .alu_a_i  (SrcA),
        .alu_b_i  (SrcB),
        .alu_p_o  (ALU_out_wire)
    );

    // Bộ cộng nhánh chuyên dụng – tách biệt với ALU chính
    // (ALU đang bận tính điều kiện so sánh trong S_EX)
    wire [31:0] branch_target = PC + ImmExt;

    // ----------------------------------------------------------------
    // Mux kết quả WB
    // ----------------------------------------------------------------
    wire [31:0] Result = (ResultSrc == 2'b00) ? ALUOut      :  // Kết quả ALU
                         (ResultSrc == 2'b01) ? MDR         :  // Dữ liệu đọc từ bộ nhớ
                         (ResultSrc == 2'b10) ? PCPlus4_reg :  // Return address (JAL/JALR)
                                                ALUOut;

    // ----------------------------------------------------------------
    // Register File
    // Chỉ ghi trong S_WB để tránh ghi nhầm
    // ----------------------------------------------------------------
    wire [31:0] rf_rdata1, rf_rdata2;
    wire        rf_we = RegWrite_en & (state == `S_WB) & ~reset;

    Register_File rf_inst (
        .clk         (clk),
        .reset       (reset),
        .read_addr_1 (rs1),
        .read_addr_2 (rs2),
        .write_addr  (rd),
        .write_data  (Result),
        .RegWrite    (rf_we),
        .read_data_1 (rf_rdata1),
        .read_data_2 (rf_rdata2)
    );

    // ----------------------------------------------------------------
    // Instruction Memory
    // ----------------------------------------------------------------
    wire [31:0] Instr;
    Instruction_Memory_v2 imem_inst (
        .read_address (PC),
        .instruction  (Instr)
    );

    // ----------------------------------------------------------------
    // Data Memory Interface
    // dmem_wen được khoá trong state S_MEM để tránh ghi ngoài ý muốn
    // ----------------------------------------------------------------
    assign dmem_addr  = ALUOut;       // Địa chỉ = kết quả ALU đã chốt
    assign dmem_wdata = B;            // Dữ liệu ghi = RS2
    assign dmem_wen   = MemWrite_en & (state == `S_MEM) & ~reset;

    // ----------------------------------------------------------------
    // FSM: Máy trạng thái điều khiển luồng thực thi
    // ----------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state       <= `S_IF;
            PC          <= 32'd0;
            IR          <= 32'h00000013; // NOP: ADDI x0, x0, 0
            A           <= 32'd0;
            B           <= 32'd0;
            ALUOut      <= 32'd0;
            MDR         <= 32'd0;
            PCPlus4_reg <= 32'd0;
        end else begin
            case (state)

                // ---- IF: Lấy lệnh, tính PC+4 ----
                `S_IF: begin
                    IR          <= Instr;           // Lấy lệnh tổ hợp từ bộ nhớ
                    PCPlus4_reg <= PC + 32'd4;      // Tính trước PC+4
                    state       <= `S_ID;
                end

                // ---- ID: Đọc thanh ghi ----
                `S_ID: begin
                    A     <= rf_rdata1;             // Chốt giá trị RS1
                    B     <= rf_rdata2;             // Chốt giá trị RS2
                    state <= `S_EX;
                end

                // ---- EX: Thực thi ALU, quyết định bước tiếp theo ----
                `S_EX: begin
                    ALUOut <= ALU_out_wire;          // Chốt kết quả ALU

                    case (opcode)
                        // Branch: cập nhật PC ngay, không cần WB
                        7'b1100011: begin
                            PC    <= ALU_out_wire[0] ? branch_target : PCPlus4_reg;
                            state <= `S_IF;
                        end
                        // Load/Store: cần truy cập bộ nhớ
                        7'b0000011, 7'b0100011: state <= `S_MEM;
                        // Các lệnh còn lại: tiến đến WB
                        default: state <= `S_WB;
                    endcase
                end

                // ---- MEM: Truy cập bộ nhớ dữ liệu ----
                // Stall ở đây cho đến khi mem_ready=1 (AXI4-Lite bridge báo xong)
                `S_MEM: begin
                    if (mem_ready) begin
                        MDR <= dmem_rdata;          // Chốt dữ liệu đọc (dùng cho Load)
                        if (opcode == 7'b0100011) begin  // Store → không cần WB
                            PC    <= PCPlus4_reg;
                            state <= `S_IF;
                        end else begin              // Load → cần WB
                            state <= `S_WB;
                        end
                    end
                    // Nếu mem_ready=0: giữ nguyên state, bus signals duy trì ổn định
                end

                // ---- WB: Ghi kết quả & cập nhật PC ----
                `S_WB: begin
                    // Việc ghi Register File được xử lý bởi rf_we (tổ hợp)
                    case (opcode)
                        7'b1101111: PC <= ALUOut;              // JAL: target = PC+imm (ALU tính ở EX)
                        7'b1100111: PC <= ALUOut & ~32'd1;     // JALR: xoá bit[0]
                        default:    PC <= PCPlus4_reg;          // Thực thi tuần tự
                    endcase
                    state <= `S_IF;
                end

                default: state <= `S_IF;
            endcase
        end
    end

endmodule


// =====================================================================
// MODULE: Immediate Extend (giống single-cycle, tương thích đầy đủ RV32I)
// =====================================================================
module extend (
    input  [2:0]  ImmSrc,
    input  [24:0] Imm,         // Imm = Instr[31:7]
    output reg [31:0] ImmExt
);
    always @(*) begin
        case (ImmSrc)
            3'b000: ImmExt = {{20{Imm[24]}}, Imm[24:13]};                            // I-type
            3'b001: ImmExt = {{20{Imm[24]}}, Imm[24:18], Imm[4:0]};                  // S-type
            3'b010: ImmExt = {{20{Imm[24]}}, Imm[0], Imm[23:18], Imm[4:1], 1'b0};   // B-type
            3'b011: ImmExt = {{12{Imm[24]}}, Imm[12:5], Imm[13], Imm[23:14], 1'b0}; // J-type
            3'b100: ImmExt = {Imm[24:5], 12'b0};                                     // U-type
            3'b101: ImmExt = {27'd0, Imm[17:13]};                                    // CSR/shamt
            default: ImmExt = 32'd0;
        endcase
    end
endmodule


// =====================================================================
// MODULE: ALU (mở rộng từ single-cycle – bổ sung shift & signed compare)
// =====================================================================
module alu (
    input  [4:0]  alu_op_i,
    input  [31:0] alu_a_i,
    input  [31:0] alu_b_i,
    output [31:0] alu_p_o
);
    reg [31:0] result_r;
    wire signed [31:0] signed_a = alu_a_i;
    wire signed [31:0] signed_b = alu_b_i;

    always @(*) begin
        case (alu_op_i)
            `ALU_ADD:                          result_r = alu_a_i + alu_b_i;
            `ALU_SUB:                          result_r = alu_a_i - alu_b_i;
            `ALU_AND:                          result_r = alu_a_i & alu_b_i;
            `ALU_OR:                           result_r = alu_a_i | alu_b_i;
            `ALU_XOR:                          result_r = alu_a_i ^ alu_b_i;
            `ALU_SHIFTL:                       result_r = alu_a_i << alu_b_i[4:0];
            `ALU_SHIFTR:                       result_r = alu_a_i >> alu_b_i[4:0];
            `ALU_SHIFTR_ARITH:                 result_r = signed_a >>> alu_b_i[4:0];
            `ALU_LESS_THAN:                    result_r = (alu_a_i < alu_b_i)   ? 32'd1 : 32'd0;
            `ALU_LESS_THAN_SIGNED:             result_r = (signed_a < signed_b) ? 32'd1 : 32'd0;
            `ALU_GREATER_THAN_OR_EQUAL:        result_r = (alu_a_i >= alu_b_i)   ? 32'd1 : 32'd0;
            `ALU_GREATER_THAN_OR_EQUAL_SIGNED: result_r = (signed_a >= signed_b) ? 32'd1 : 32'd0;
            `ALU_EQUAL:                        result_r = (alu_a_i == alu_b_i)  ? 32'd1 : 32'd0;
            `ALU_NOT_EQUAL:                    result_r = (alu_a_i != alu_b_i)  ? 32'd1 : 32'd0;
            `ALU_LOAD_UPPER:                   result_r = alu_b_i;
            default:                           result_r = alu_a_i + alu_b_i;
        endcase
    end
    assign alu_p_o = result_r;
endmodule


// =====================================================================
// MODULE: Register File (x0 cứng = 0, đọc tổ hợp, ghi đồng bộ)
// =====================================================================
module Register_File (
    input        clk,
    input        reset,
    input  [4:0] read_addr_1,
    input  [4:0] read_addr_2,
    input  [4:0] write_addr,
    input  [31:0] write_data,
    input        RegWrite,
    output reg [31:0] read_data_1,
    output reg [31:0] read_data_2
);
    reg [31:0] Regfile [0:31];
    integer k;

    // Đọc tổ hợp (FWFT behavior)
    always @(*) begin
        read_data_1 = (read_addr_1 == 5'd0) ? 32'd0 : Regfile[read_addr_1];
        read_data_2 = (read_addr_2 == 5'd0) ? 32'd0 : Regfile[read_addr_2];
    end

    // Ghi đồng bộ
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (k = 0; k < 32; k = k + 1)
                Regfile[k] <= 32'd0;
        end else if (RegWrite && (write_addr != 5'd0)) begin
            Regfile[write_addr] <= write_data;
        end
    end
endmodule


// =====================================================================
// MODULE: Instruction Memory – Chương trình test SPI Controller
// Bố cục chương trình (byte address):
//   0x00 – 0x0C  : Khởi tạo SPI (CTRL, BAUD, counter)
//   0x18         : LOOP_TX  – ghi dữ liệu vào TXDATA
//   0x1C         : POLL_BUSY – đọc STATUS, chờ hết busy
//   0x28 – 0x30  : Đọc RXDATA, lưu kết quả
//   0x34         : Tăng counter, nhảy về LOOP_TX
// =====================================================================
module Instruction_Memory_v2 (
    input  [31:0] read_address,
    output [31:0] instruction
);
    reg [31:0] Imemory [0:255];
    integer k;

    assign instruction = Imemory[read_address[31:2]];

    initial begin
        for (k = 0; k < 256; k = k + 1)
            Imemory[k] = 32'h00000013; // NOP (ADDI x0, x0, 0)

        // ---- Khởi tạo SPI Controller ----
        // x10 = 0x00010000 (SPI base address)
        Imemory[0]  = 32'h00010537; // lui   x10, 0x10
        // x11 = 0 (CTRL: CPOL=0, CPHA=0, MSB-first)
        Imemory[1]  = 32'h00000593; // addi  x11, x0, 0
        // mem[x10+0] = 0  → SPI_CTRL
        Imemory[2]  = 32'h00B52023; // sw    x11, 0(x10)
        // x12 = 49 (Baud divisor: f_sclk = f_clk / (49+1))
        Imemory[3]  = 32'h03100613; // addi  x12, x0, 49
        // mem[x10+4] = 49  → SPI_BAUD
        Imemory[4]  = 32'h00C52223; // sw    x12, 4(x10)
        // x13 = 0 (TX data counter)
        Imemory[5]  = 32'h00000693; // addi  x13, x0, 0

        // ---- LOOP_TX (PC = 0x18, Imemory[6]) ----
        // mem[x10+8] = x13  → SPI_TXDATA (kích hoạt truyền)
        Imemory[6]  = 32'h00D52423; // sw    x13, 8(x10)

        // ---- POLL_BUSY (PC = 0x1C, Imemory[7]) ----
        // x14 = mem[x10+16]  → đọc SPI_STATUS
        Imemory[7]  = 32'h01052703; // lw    x14, 16(x10)
        // x14 = x14 & 1  → lấy bit BUSY
        Imemory[8]  = 32'h00177713; // andi  x14, x14, 1
        // if (x14 != 0) goto POLL_BUSY (offset = -8)
        Imemory[9]  = 32'hFE071CE3; // bne   x14, x0, -8

        // ---- Thu thập kết quả ----
        // x15 = mem[x10+12]  → đọc SPI_RXDATA
        Imemory[10] = 32'h00C52783; // lw    x15, 12(x10)
        // mem[0x0] = x15  → lưu kết quả nhận vào SRAM
        Imemory[11] = 32'h00F02023; // sw    x15, 0(x0)
        // x13 = x13 + 1  → tăng TX counter
        Imemory[12] = 32'h00168693; // addi  x13, x13, 1
        // goto LOOP_TX (offset = -28, PC = 0x34)
        Imemory[13] = 32'hFE5FF06F; // jal   x0, -28
    end
endmodule
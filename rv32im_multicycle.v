`timescale 1ns / 1ps
// =====================================================================
// RV32IMZicsr Multi-Cycle Processor – Single Combined File
// 54 instructions: RV32I(40) + RV32M(8) + Zicsr(6)
// FSM: S_IF S_ID S_EX S_MEM S_WB S_DIV
// Sub-modules (adapted from Hari545543/RISC-V-RV32I):
//   control_unit  alu_riscv  imm_gen  branch_unit  load_unit  store_unit
// =====================================================================

// ── ALU Op Codes ──────────────────────────────────────────────────────
`define ADD     4'd0
`define SUB     4'd1
`define XOR     4'd2
`define OR      4'd3
`define AND     4'd4
`define SLL     4'd5
`define SRL     4'd6
`define SRA     4'd7
`define SLT     4'd8
`define SLTU    4'd9
`define ERR     4'd15

// ── Branch funct3 ─────────────────────────────────────────────────────
`define BEQ     3'b000
`define BNE     3'b001
`define BLT     3'b100
`define BGE     3'b101
`define BLTU    3'b110
`define BGEU    3'b111

// ── Instruction Formats ───────────────────────────────────────────────
`define I_type_load   3'b000
`define I_type        3'b001
`define S_type        3'b010
`define R_type        3'b011
`define U_type_LUI    3'b100
`define J_type        3'b101
`define B_type        3'b110
`define U_type_AUIPC  3'b111

// ── Opcodes ───────────────────────────────────────────────────────────
`define OP_LUI    7'b0110111
`define OP_AUIPC  7'b0010111
`define OP_JAL    7'b1101111
`define OP_JALR   7'b1100111
`define OP_BRANCH 7'b1100011
`define OP_LOAD   7'b0000011
`define OP_STORE  7'b0100011
`define OP_IMM    7'b0010011
`define OP_REG    7'b0110011
`define OP_FENCE  7'b0001111
`define OP_SYSTEM 7'b1110011

// ── FSM States ────────────────────────────────────────────────────────
`define S_IF   3'd0
`define S_ID   3'd1
`define S_EX   3'd2
`define S_MEM  3'd3
`define S_WB   3'd4
`define S_DIV  3'd5

// ── ResultSrc ─────────────────────────────────────────────────────────
`define RES_ALU  2'b00
`define RES_MDR  2'b01
`define RES_PC4  2'b10
`define RES_CSR  2'b11

// ── Trap Codes ────────────────────────────────────────────────────────
`define CAUSE_ILLEGAL  32'h00000002
`define CAUSE_EBREAK   32'h00000003
`define CAUSE_ECALL_M  32'h0000000B

// ── CSR Addresses ─────────────────────────────────────────────────────
`define CSR_MSTATUS  12'h300
`define CSR_MIE      12'h304
`define CSR_MTVEC    12'h305
`define CSR_MSCRATCH 12'h340
`define CSR_MEPC     12'h341
`define CSR_MCAUSE   12'h342
`define CSR_MTVAL    12'h343
`define CSR_MIP      12'h344
`define CSR_MCYCLE   12'hB00
`define CSR_MCYCLEH  12'hB80
`define CSR_MINSTRET 12'hB02
`define CSR_MISA     12'h301
`define CSR_MHARTID  12'hF14

// =====================================================================
// MODULE: control_unit
// Hàm from Hari545543 repo + thêm is_muldiv, is_csr
// =====================================================================
module control_unit (
    input  wire [6:0] opcode_in,
    input  wire [2:0] funct3_in,
    input  wire [6:0] funct7_in,
    output reg        RegWrite_en, MemWrite_en, ALUSrc, UIPC_add,
    output reg [1:0]  ResultSrc,
    output reg [3:0]  ALUCtrl,
    output reg [2:0]  ImmSrc,
    output reg        is_branch, is_jalr, is_muldiv, is_csr,
    output reg        is_trap,
    output reg [31:0] trap_cause
);
    wire r_type       = (opcode_in == `OP_REG);
    wire i_type       = (opcode_in == `OP_IMM);
    wire i_type_load  = (opcode_in == `OP_LOAD);
    wire s_type       = (opcode_in == `OP_STORE);
    wire b_type       = (opcode_in == `OP_BRANCH);
    wire j_type_jal   = (opcode_in == `OP_JAL);
    wire j_type_jalr  = (opcode_in == `OP_JALR);
    wire u_type_lui   = (opcode_in == `OP_LUI);
    wire u_type_auipc = (opcode_in == `OP_AUIPC);
    wire sys_type     = (opcode_in == `OP_SYSTEM);
    wire fence_type   = (opcode_in == `OP_FENCE);
    wire is_mext      = r_type && (funct7_in == 7'b0000001);

    function [3:0] for_aluop_rtype;
        input [2:0] f3; input f7;
        begin
            case (f3)
                3'b000: for_aluop_rtype = f7 ? `SUB : `ADD;
                3'b001: for_aluop_rtype = `SLL;
                3'b010: for_aluop_rtype = `SLT;
                3'b011: for_aluop_rtype = `SLTU;
                3'b100: for_aluop_rtype = `XOR;
                3'b101: for_aluop_rtype = f7 ? `SRA : `SRL;
                3'b110: for_aluop_rtype = `OR;
                3'b111: for_aluop_rtype = `AND;
                default: for_aluop_rtype = `ADD;
            endcase
        end
    endfunction

    function [3:0] for_aluop_itype;
        input [2:0] f3; input f7;
        begin
            case (f3)
                3'b000: for_aluop_itype = `ADD;
                3'b001: for_aluop_itype = `SLL;
                3'b010: for_aluop_itype = `SLT;
                3'b011: for_aluop_itype = `SLTU;
                3'b100: for_aluop_itype = `XOR;
                3'b101: for_aluop_itype = f7 ? `SRA : `SRL;
                3'b110: for_aluop_itype = `OR;
                3'b111: for_aluop_itype = `AND;
                default: for_aluop_itype = `ADD;
            endcase
        end
    endfunction

    function [2:0] get_immsrc;
        input [6:0] op;
        begin
            case (op)
                `OP_IMM, `OP_LOAD, `OP_JALR: get_immsrc = `I_type;
                `OP_STORE:  get_immsrc = `S_type;
                `OP_BRANCH: get_immsrc = `B_type;
                `OP_JAL:    get_immsrc = `J_type;
                `OP_LUI, `OP_AUIPC: get_immsrc = `U_type_LUI;
                default:    get_immsrc = `I_type;
            endcase
        end
    endfunction

    always @(*) begin
        RegWrite_en=0; MemWrite_en=0; ALUSrc=0; UIPC_add=0;
        ResultSrc=`RES_ALU; ALUCtrl=`ADD; ImmSrc=`I_type;
        is_branch=0; is_jalr=0; is_muldiv=0; is_csr=0;
        is_trap=0; trap_cause=0;

        if (r_type) begin
            RegWrite_en = 1;
            if (is_mext) is_muldiv = 1;
            else ALUCtrl = for_aluop_rtype(funct3_in, funct7_in[5]);
        end
        else if (i_type) begin
            RegWrite_en=1; ALUSrc=1; ImmSrc=`I_type;
            ALUCtrl = for_aluop_itype(funct3_in, funct7_in[5]);
        end
        else if (i_type_load) begin
            RegWrite_en=1; ALUSrc=1; ImmSrc=`I_type;
            ALUCtrl=`ADD; ResultSrc=`RES_MDR;
        end
        else if (s_type) begin
            ALUSrc=1; ImmSrc=`S_type; ALUCtrl=`ADD; MemWrite_en=1;
        end
        else if (b_type) begin
            ImmSrc=`B_type; is_branch=1;
        end
        else if (j_type_jal) begin
            RegWrite_en=1; ImmSrc=`J_type; UIPC_add=1;
            ALUSrc=1; ALUCtrl=`ADD; ResultSrc=`RES_PC4;
        end
        else if (j_type_jalr) begin
            RegWrite_en=1; ALUSrc=1; ALUCtrl=`ADD;
            is_jalr=1; ResultSrc=`RES_PC4;
        end
        else if (u_type_lui) begin
            RegWrite_en=1; ImmSrc=`U_type_LUI; ALUSrc=1; ALUCtrl=`ADD;
        end
        else if (u_type_auipc) begin
            RegWrite_en=1; ImmSrc=`U_type_AUIPC; UIPC_add=1; ALUSrc=1; ALUCtrl=`ADD;
        end
        else if (fence_type) begin
            // NOP
        end
        else if (sys_type) begin
            case (funct3_in)
                3'b000: begin is_trap=1; trap_cause=`CAUSE_ECALL_M; end
                3'b001, 3'b010, 3'b011,
                3'b101, 3'b110, 3'b111: begin
                    RegWrite_en=1; is_csr=1; ResultSrc=`RES_CSR;
                end
                default: begin is_trap=1; trap_cause=`CAUSE_ILLEGAL; end
            endcase
        end
        else begin
            is_trap=1; trap_cause=`CAUSE_ILLEGAL;
        end

        if (!r_type && !fence_type && !sys_type)
            ImmSrc = get_immsrc(opcode_in);
    end
endmodule


// =====================================================================
// MODULE: alu_riscv
// =====================================================================
module alu_riscv (
    input  wire [31:0] operand_1, operand_2,
    input  wire [3:0]  aluop,
    output reg  [31:0] out
);
    wire [31:0] addr_in=operand_1+operand_2, sub_in=operand_1-operand_2;
    wire [31:0] xor_in=operand_1^operand_2, or_in=operand_1|operand_2;
    wire [31:0] and_in=operand_1&operand_2;
    wire [31:0] sll_in=operand_1<<operand_2[4:0], srl_in=operand_1>>operand_2[4:0];
    wire signed [31:0] sop1=operand_1;
    wire [31:0] sra_in = sop1>>>operand_2[4:0];
    wire signed [31:0] rs1s=operand_1, rs2s=operand_2;
    wire slt_in=(rs1s<rs2s), sltu_in=(operand_1<operand_2);

    always @(*) begin
        out = 32'd0;
        case (aluop)
            `ADD:  out=addr_in; `SUB:  out=sub_in;
            `XOR:  out=xor_in;  `OR:   out=or_in;
            `AND:  out=and_in;  `SLL:  out=sll_in;
            `SRL:  out=srl_in;  `SRA:  out=sra_in;
            `SLT:  out={31'b0,slt_in}; `SLTU: out={31'b0,sltu_in};
            default: out=addr_in;
        endcase
    end
endmodule


// =====================================================================
// MODULE: imm_gen 
// =====================================================================
module imm_gen (
    input  wire [24:0] instr,
    input  wire [2:0]  imm_mux,
    output reg  [31:0] imm
);
    wire [31:0] i_t={{20{instr[24]}},instr[24:13]};
    wire [31:0] s_t={{20{instr[24]}},instr[24:18],instr[4:0]};
    wire [31:0] b_t={{20{instr[24]}},instr[0],instr[23:18],instr[4:1],1'b0};
    wire [31:0] j_t={{12{instr[24]}},instr[12:5],instr[13],instr[23:14],1'b0};
    wire [31:0] u_t={instr[24:5],12'h000};
    always @(*) begin
        case (imm_mux)
            `I_type,`I_type_load: imm=i_t;
            `S_type:              imm=s_t;
            `B_type:              imm=b_t;
            `J_type:              imm=j_t;
            `U_type_LUI,`U_type_AUIPC: imm=u_t;
            default:              imm=i_t;
        endcase
    end
endmodule


// =====================================================================
// MODULE: branch_unit 
// =====================================================================
module branch_unit (
    input  wire [2:0]  funct3_in,
    input  wire [31:0] source_1, source_2,
    input  wire        is_branch,
    output wire        branch_taken
);
    wire signed [31:0] s1=source_1, s2=source_2;
    wire beq=(source_1==source_2), bne=(source_1!=source_2);
    wire blt=(s1<s2), bge=(s1>=s2);
    wire bltu=(source_1<source_2), bgeu=(source_1>=source_2);
    reg br;
    always @(*) begin
        br=0;
        case (funct3_in)
            `BEQ:br=beq; `BNE:br=bne; `BLT:br=blt;
            `BGE:br=bge; `BLTU:br=bltu; `BGEU:br=bgeu;
            default:br=0;
        endcase
    end
    assign branch_taken = is_branch & br;
endmodule


// =====================================================================
// MODULE: load_unit
// =====================================================================
module load_unit (
    input  wire [31:0] data_in,
    input  wire [2:0]  load_funct3_in,
    input  wire [1:0]  byte_offset,
    output reg  [31:0] load_output
);
    reg [7:0] bp; reg [15:0] hp; reg bs, hs;
    always @(*) begin
        case (byte_offset)
            2'b00:bp=data_in[7:0]; 2'b01:bp=data_in[15:8];
            2'b10:bp=data_in[23:16]; default:bp=data_in[31:24];
        endcase
        hp = byte_offset[1] ? data_in[31:16] : data_in[15:0];
        bs = load_funct3_in[2] ? 1'b0 : bp[7];
        hs = load_funct3_in[2] ? 1'b0 : hp[15];
        case (load_funct3_in[1:0])
            2'b00: load_output={{24{bs}},bp};
            2'b01: load_output={{16{hs}},hp};
            2'b10: load_output=data_in;
            default: load_output=data_in;
        endcase
    end
endmodule


// =====================================================================
// MODULE: store_unit 
// =====================================================================
module store_unit (
    input  wire [31:0] rs2_in,
    input  wire [1:0]  funct3_in, byte_offset,
    output reg  [31:0] rs2_out,
    output reg  [3:0]  wb_mask_out
);
    always @(*) begin
        rs2_out=rs2_in; wb_mask_out=4'b1111;
        case (funct3_in)
            2'b00: case (byte_offset)
                2'd0:begin rs2_out={24'b0,rs2_in[7:0]};       wb_mask_out=4'b0001;end
                2'd1:begin rs2_out={16'b0,rs2_in[7:0],8'b0};  wb_mask_out=4'b0010;end
                2'd2:begin rs2_out={8'b0,rs2_in[7:0],16'b0};  wb_mask_out=4'b0100;end
                2'd3:begin rs2_out={rs2_in[7:0],24'b0};        wb_mask_out=4'b1000;end
            endcase
            2'b01: if(!byte_offset[1])begin rs2_out={16'b0,rs2_in[15:0]};wb_mask_out=4'b0011;end
                   else               begin rs2_out={rs2_in[15:0],16'b0};wb_mask_out=4'b1100;end
            2'b10: begin rs2_out=rs2_in; wb_mask_out=4'b1111; end
            default: begin rs2_out=rs2_in; wb_mask_out=4'b1111; end
        endcase
    end
endmodule


// =====================================================================
// MODULE: Datapath_Multi_cycle_Processor_RISC_V 
// =====================================================================
module Datapath_Multi_cycle_Processor_RISC_V (
    input  wire        clk, reset,
    output wire [31:0] PC_out, ALUResult_out,
    output wire [2:0]  state_dbg,
    output wire [31:0] dmem_addr, dmem_wdata,
    output wire [3:0]  dmem_wstrb,
    input  wire [31:0] dmem_rdata,
    input  wire        mem_ready,
    output wire        trap_o,
    output wire [31:0] mcause_o, mepc_o
);
    reg [31:0] PC, IR, A, B, ALUOut, MDR, PCPlus4_reg;
    reg [2:0]  state;
    reg [31:0] mcause_r, mepc_r;
    reg        trap_r;

    assign PC_out=PC; assign ALUResult_out=ALUOut;
    assign state_dbg=state; assign trap_o=trap_r;
    assign mcause_o=mcause_r; assign mepc_o=mepc_r;

    // ── CSR Registers ───────────────────────────────────────────────
    reg [31:0] csr_mstatus, csr_mtvec, csr_mscratch;
    reg [31:0] csr_mepc, csr_mcause, csr_mtval, csr_mie;
    reg [63:0] csr_mcycle, csr_minstret;

    always @(posedge clk or posedge reset)
        if (reset) csr_mcycle <= 64'd0;
        else       csr_mcycle <= csr_mcycle + 64'd1;

    always @(posedge clk or posedge reset) begin
        if (reset) csr_minstret <= 64'd0;
        else if ((state==`S_WB) ||
                 (state==`S_EX  && IR[6:0]==`OP_BRANCH) ||
                 (state==`S_MEM && IR[6:0]==`OP_STORE && mem_ready))
            csr_minstret <= csr_minstret + 64'd1;
    end

    wire [11:0] csr_addr = IR[31:20];
    reg  [31:0] csr_rdata;
    always @(*) begin
        case (csr_addr)
            `CSR_MSTATUS:  csr_rdata=csr_mstatus;
            `CSR_MIE:      csr_rdata=csr_mie;
            `CSR_MTVEC:    csr_rdata=csr_mtvec;
            `CSR_MSCRATCH: csr_rdata=csr_mscratch;
            `CSR_MEPC:     csr_rdata=csr_mepc;
            `CSR_MCAUSE:   csr_rdata=csr_mcause;
            `CSR_MTVAL:    csr_rdata=csr_mtval;
            `CSR_MCYCLE:   csr_rdata=csr_mcycle[31:0];
            `CSR_MCYCLEH:  csr_rdata=csr_mcycle[63:32];
            `CSR_MINSTRET: csr_rdata=csr_minstret[31:0];
            `CSR_MISA:     csr_rdata=32'h4000_1100;
            `CSR_MHARTID:  csr_rdata=32'd0;
            default:       csr_rdata=32'd0;
        endcase
    end

    wire [31:0] csr_zimm    = {27'd0, IR[19:15]};
    wire [31:0] csr_wval_src = IR[14] ? csr_zimm : A;
    reg  [31:0] csr_wdata;
    always @(*) begin
        case (IR[13:12])
            2'b01: csr_wdata = csr_wval_src;
            2'b10: csr_wdata = csr_rdata | csr_wval_src;
            2'b11: csr_wdata = csr_rdata & ~csr_wval_src;
            default: csr_wdata = csr_rdata;
        endcase
    end

    wire [6:0] opcode=IR[6:0], funct7=IR[31:25];
    wire [2:0] funct3=IR[14:12];
    wire [4:0] rs1=IR[19:15], rs2=IR[24:20], rd=IR[11:7];

    wire        RegWrite_en, MemWrite_en, ALUSrc, UIPC_add;
    wire [1:0]  ResultSrc;
    wire [3:0]  ALUCtrl;
    wire [2:0]  ImmSrc;
    wire        is_branch, is_jalr, is_muldiv, is_csr;
    wire        is_trap_ctrl;
    wire [31:0] trap_cause_ctrl;

    control_unit u_ctrl (
        .opcode_in(opcode), .funct3_in(funct3), .funct7_in(funct7),
        .RegWrite_en(RegWrite_en), .MemWrite_en(MemWrite_en),
        .ALUSrc(ALUSrc), .UIPC_add(UIPC_add), .ResultSrc(ResultSrc),
        .ALUCtrl(ALUCtrl), .ImmSrc(ImmSrc),
        .is_branch(is_branch), .is_jalr(is_jalr),
        .is_muldiv(is_muldiv), .is_csr(is_csr),
        .is_trap(is_trap_ctrl), .trap_cause(trap_cause_ctrl)
    );

    wire is_trap   = is_trap_ctrl;
    wire [31:0] trap_cause = (opcode==`OP_SYSTEM && funct3==3'b000 && IR[20])
                              ? `CAUSE_EBREAK : trap_cause_ctrl;

    wire [31:0] ImmExt;
    imm_gen u_immgen (.instr(IR[31:7]), .imm_mux(ImmSrc), .imm(ImmExt));

    wire is_lui = (opcode==`OP_LUI);
    wire [31:0] SrcA = UIPC_add ? PC : (is_lui ? 32'd0 : A);
    wire [31:0] SrcB = ALUSrc ? ImmExt : B;

    wire [31:0] ALU_out_wire;
    alu_riscv u_alu (.operand_1(SrcA), .operand_2(SrcB),
                     .aluop(ALUCtrl), .out(ALU_out_wire));

    wire [31:0] branch_target = PC + ImmExt;

    wire branch_taken;
    branch_unit u_branch (.funct3_in(funct3), .source_1(A), .source_2(B),
                          .is_branch(is_branch), .branch_taken(branch_taken));

    wire [31:0] load_result;
    load_unit u_load (.data_in(dmem_rdata), .load_funct3_in(funct3),
                      .byte_offset(ALUOut[1:0]), .load_output(load_result));

    wire [31:0] store_wdata; wire [3:0] store_wstrb;
    store_unit u_store (.rs2_in(B), .funct3_in(funct3[1:0]),
                        .byte_offset(ALUOut[1:0]),
                        .rs2_out(store_wdata), .wb_mask_out(store_wstrb));

    assign dmem_addr  = ALUOut;
    assign dmem_wdata = store_wdata;
    assign dmem_wstrb = (MemWrite_en && state==`S_MEM && !reset) ? store_wstrb : 4'b0;

    // ── RV32M Multiply (combinational) ──────────────────────────────
    wire signed [31:0] msa=A, msb=B;
    wire [63:0] mul_uu = A * B;
    wire signed [63:0] mul_ss = msa * msb;
    wire signed [63:0] mul_su = $signed(A) * $unsigned(B);
    reg [31:0] mul_result;
    always @(*) begin
        case (funct3)
            3'b000: mul_result=mul_uu[31:0];
            3'b001: mul_result=mul_ss[63:32];
            3'b010: mul_result=mul_su[63:32];
            3'b011: mul_result=mul_uu[63:32];
            default: mul_result=mul_uu[31:0];
        endcase
    end

    // ── RV32M Divider registers ──────────────────────────────────────
   // ── RV32M Divider registers ──────────────────────────────────────
    reg [31:0] div_quot, div_rem;
    reg [5:0]  div_cnt;
    reg [63:0] div_acc;        // [63:32]=partial remainder, [31:0]=dividend/quotient
    reg [31:0] div_dvsr;
    reg        div_sign_q, div_sign_r;

    // shifted partial-remainder minus divisor
    wire [31:0] div_sub = div_acc[62:31] - div_dvsr;

    // ── WB Mux ──────────────────────────────────────────────────────
    wire [31:0] mext_res = funct3[2] ? (funct3[1] ? div_rem : div_quot) : mul_result;
    wire [31:0] Result =
        (ResultSrc==`RES_MDR) ? MDR         :
        (ResultSrc==`RES_PC4) ? PCPlus4_reg :
        (ResultSrc==`RES_CSR) ? csr_rdata   :
        is_muldiv             ? mext_res    :
                                ALUOut;

    wire [31:0] rf_rdata1, rf_rdata2;
    wire rf_we = RegWrite_en & (state==`S_WB) & ~reset & ~is_trap;

    Register_File u_rf (
        .clk(clk), .reset(reset),
        .read_addr_1(rs1), .read_addr_2(rs2),
        .write_addr(rd), .write_data(Result), .RegWrite(rf_we),
        .read_data_1(rf_rdata1), .read_data_2(rf_rdata2)
    );

    wire [31:0] Instr;
    Instruction_Memory_v2 u_imem (.read_address(PC), .instruction(Instr));

    // ── FSM ─────────────────────────────────────────────────────────
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state<=`S_IF; PC<=0; IR<=32'h0000_0013;
            A<=0; B<=0; ALUOut<=0; MDR<=0; PCPlus4_reg<=0;
            mcause_r<=0; mepc_r<=0; trap_r<=0;
            div_quot<=0; div_rem<=0; div_cnt<=0;
            div_acc<=0; div_dvsr<=0; div_sign_q<=0; div_sign_r<=0;
            csr_mstatus<=32'h0000_1800; csr_mtvec<=0; csr_mscratch<=0;
            csr_mepc<=0; csr_mcause<=0; csr_mtval<=0; csr_mie<=0;
        end else begin
            trap_r <= 0;
            case (state)
                `S_IF: begin
                    IR<=Instr; PCPlus4_reg<=PC+32'd4; state<=`S_ID;
                end
                `S_ID: begin
                    A<=rf_rdata1; B<=rf_rdata2;
                    if (is_trap) begin
                        mcause_r<=trap_cause; mepc_r<=PC;
                        trap_r<=1; PC<=PCPlus4_reg; state<=`S_IF;
                    end else state<=`S_EX;
                end
                `S_EX: begin
                    ALUOut <= ALU_out_wire;
                    case (opcode)
                        `OP_BRANCH: begin
                            PC <= branch_taken ? branch_target : PCPlus4_reg;
                            state <= `S_IF;
                        end
                        `OP_LOAD, `OP_STORE: state <= `S_MEM;
                        `OP_REG: begin
                            if (is_muldiv && funct3[2]) begin
                                // DIV/REM: setup và vào S_DIV
                                if (B == 32'd0) begin
                                    div_quot <= 32'hFFFF_FFFF;
                                    div_rem  <= A;
                                    state    <= `S_WB;
                                end else begin
                                    if (!funct3[0]) begin // signed
                                        div_sign_q <= A[31]^B[31];
                                        div_sign_r <= A[31];
                                        div_acc  <= {32'd0,(A[31]?-A:A)};
                                        div_dvsr <=        (B[31]?-B:B);
                                    end else begin // unsigned
                                        div_sign_q <= 0; div_sign_r <= 0;
                                        div_acc <= {32'd0, A}; div_dvsr <= B;
                                    end
                                    div_quot<=0; div_cnt<=6'd32; state<=`S_DIV;
                                end
                            end else state <= `S_WB;
                        end
                        default: state <= `S_WB;
                    endcase
                end
                `S_DIV: begin
                    if (div_cnt > 0) begin
                        // shift the 64-bit accumulator left by 1; quotient bit lands in acc[0]
                        if (div_acc[62:31] >= div_dvsr)
                            div_acc <= { div_sub, div_acc[30:0], 1'b1 };
                        else
                            div_acc <= { div_acc[62:0], 1'b0 };
                        div_cnt <= div_cnt - 1;
                    end else begin
                        div_rem  <= div_sign_r ? -div_acc[63:32] : div_acc[63:32];
                        div_quot <= div_sign_q ? -div_acc[31:0]  : div_acc[31:0];
                        state    <= `S_WB;
                    end
                end
                `S_MEM: begin
                    if (mem_ready) begin
                        MDR <= load_result;
                        if (opcode==`OP_STORE) begin PC<=PCPlus4_reg; state<=`S_IF; end
                        else state <= `S_WB;
                    end
                end
                `S_WB: begin
                    if (is_csr && !reset) begin
                        case (csr_addr)
                            `CSR_MSTATUS:  csr_mstatus  <= csr_wdata;
                            `CSR_MIE:      csr_mie      <= csr_wdata;
                            `CSR_MTVEC:    csr_mtvec    <= csr_wdata;
                            `CSR_MSCRATCH: csr_mscratch <= csr_wdata;
                            `CSR_MEPC:     csr_mepc     <= csr_wdata;
                            `CSR_MCAUSE:   csr_mcause   <= csr_wdata;
                            `CSR_MTVAL:    csr_mtval    <= csr_wdata;
                            default: ;
                        endcase
                    end
                    case (opcode)
                        `OP_JAL:  PC <= ALUOut;
                        `OP_JALR: PC <= ALUOut & ~32'd1;
                        default:  PC <= PCPlus4_reg;
                    endcase
                    state <= `S_IF;
                end
                default: state <= `S_IF;
            endcase
        end
    end
endmodule


// =====================================================================
// MODULE: Register_File
// =====================================================================
module Register_File (
    input  wire clk, reset,
    input  wire [4:0]  read_addr_1, read_addr_2, write_addr,
    input  wire [31:0] write_data,
    input  wire        RegWrite,
    output reg  [31:0] read_data_1, read_data_2
);
    reg [31:0] Regfile [0:31]; integer k;
    always @(*) begin
        read_data_1 = (read_addr_1==0) ? 32'd0 : Regfile[read_addr_1];
        read_data_2 = (read_addr_2==0) ? 32'd0 : Regfile[read_addr_2];
    end
    always @(posedge clk or posedge reset) begin
        if (reset) for (k=0;k<32;k=k+1) Regfile[k]<=32'd0;
        else if (RegWrite && write_addr!=0) Regfile[write_addr]<=write_data;
    end
endmodule


// =====================================================================
// MODULE: Instruction_Memory_v2 (Đã cập nhật truyền 32'h11111111)
// =====================================================================
// =====================================================================
// MODULE: Instruction_Memory_v2 (Chương trình tự kiểm thử toàn diện RV32I)
// =====================================================================
module Instruction_Memory_v2 (
    input  wire [31:0] read_address,
    output wire [31:0] instruction
);
    reg [31:0] Imemory [0:255]; integer k;
    assign instruction = Imemory[read_address[31:2]];

    initial begin
        // Khởi tạo bộ nhớ mặc định là lệnh NOP (addi x0, x0, 0)
        for (k=0; k<256; k=k+1) Imemory[k] = 32'h0000_0013;

        // ── 1. KHỞI TẠO THANH GHI ───────────────────────────────────
        Imemory[0]  = 32'h00A00293; // addi  x5, x0, 10      -> x5 = 10
        Imemory[1]  = 32'h00300313; // addi  x6, x0, 3       -> x6 = 3

        // ── 2. KIỂM THỬ SỐ HỌC & LOGIC (add, sub, and, or, xor, sll, srl) ────
        Imemory[2]  = 32'h006283B3; // add   x7, x5, x6      -> x7 = 10 + 3 = 13
        Imemory[3]  = 32'h40628433; // sub   x8, x5, x6      -> x8 = 10 - 3 = 7
        Imemory[4]  = 32'h0062F4B3; // and   x9, x5, x6      -> x9 = 10 & 3 = 2
        Imemory[5]  = 32'h0062E533; // or    x10, x5, x6     -> x10 = 10 | 3 = 11
        Imemory[6]  = 32'h0062C5B3; // xor   x11, x5, x6     -> x11 = 10 ^ 3 = 9
        Imemory[7]  = 32'h00629633; // sll   x12, x5, x6     -> x12 = 10 << 3 = 80
        Imemory[8]  = 32'h0062D6B3; // srl   x13, x5, x6     -> x13 = 10 >> 3 = 1

        // ── 3. KIỂM THỬ GHI/ĐỌC TRỰC TIẾP BYTE & HALFWORD (sb, sh, lbu, lhu) ──
        Imemory[9]  = 32'h00000713; // addi  x14, x0, 0      -> x14 = 0 (Địa chỉ cơ sở SRAM)
        Imemory[10] = 32'h05A00793; // addi  x15, x0, 90     -> x15 = 90 (0x5A)
        Imemory[11] = 32'h00F70023; // sb    x15, 0(x14)     -> Lưu 1 Byte (90) vào SRAM[0]
        
        Imemory[12] = 32'h3FF00813; // addi  x16, x0, 1023   -> x16 = 1023 (12'h3FF)
        Imemory[13] = 32'h01071123; // sh    x16, 2(x14)     -> Lưu Halfword (1023) vào SRAM địa chỉ 2

        Imemory[14] = 32'h00074883; // lbu   x17, 0(x14)     -> Đọc ngược lại 1 Byte không dấu -> x17 = 90
        Imemory[15] = 32'h00275903; // lhu   x18, 2(x14)     -> Đọc ngược lại Halfword không dấu -> x18 = 1023

        // ── 4. KIỂM THỬ ĐIỀU KIỆN RẼ NHÁNH & NHẢY (beq, blt, jal) ────────────
        Imemory[16] = 32'h01178463; // beq   x15, x17, 8     -> Nếu x15 == x17 (90 == 90), nhảy qua 1 lệnh
        Imemory[17] = 32'h00000393; // addi  x7, x0, 0       -> (Lệnh này sẽ bị bỏ qua nếu BEQ hoạt động đúng)

        Imemory[18] = 32'h00734463; // blt   x6, x5, 8       -> Nếu x6 < x5 (3 < 10), nhảy qua 1 lệnh
        Imemory[19] = 32'h00000413; // addi  x8, x0, 0       -> (Lệnh này sẽ bị bỏ qua nếu BLT hoạt động đúng)

        // ── 5. TỔNG HỢP KẾT QUẢ GHI VÀO SRAM[0] ĐỂ QUAN SÁT ─────────────────
        Imemory[20] = 32'h00740933; // add   x18, x8, x7     -> x18 = 7 + 13 = 20 (0x14)
        Imemory[21] = 32'h00000713; // addi  x14, x0, 0
        Imemory[22] = 32'h01272023; // sw    x18, 0(x14)     -> Ghi kết quả cuối cùng (20) vào SRAM[0]

        Imemory[23] = 32'hFE00006F; // jal   x0, -32         -> Quay ngược lại Imemory[15] để lặp vô hạn
    end
endmodule


`timescale 1ns / 1ps

// ========================================

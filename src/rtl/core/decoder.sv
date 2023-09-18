/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/13/2023
 *
 * ------------------------------------------------------------------------------------------------
 * Decoder: Instruction Decoder
 * ------------------------------------------------------------------------------------------------
 */

// Notes: We only detect illegal instruction detection on opcode.
// If other part of the instruction (such as funct3) is illegal, we don't detect that
// and the instruction might be treated as NOP depending on the encoding

`include "config.svh"
`include "core.svh"
`include "riscv_isa.svh"

module decoder #(
    parameter SUPPORT_ZICSR = 1,
    parameter SUPPORT_TRAP = 1
) (
    input logic [`XLEN-1:0]             instruction,
    // contrl signal to downstrem pipeline stage
    output logic [`ALU_OP_WIDTH-1:0]    dec_alu_opcode,     // alu opcode
    output logic [`ALU_SRC1_WIDTH-1:0]  dec_alu_src1_sel,   // alu src1 select
    output logic                        dec_alu_src2_sel_imm,   // alu src2 select
    output logic                        dec_branch,         // branch instruction
    output logic [`BRANCH_OP_WIDTH-1:0] dec_branch_opcode,  // branch opcode
    output logic                        dec_jump,           // jump instructions
    output logic                        dec_mem_read,       // memory read
    output logic                        dec_mem_write,      // memory write
    output logic [`MEM_OP_WIDTH-1:0]    dec_mem_opcode,     // memory operation
    output logic                        dec_unsign,         // unsigned branch instruction or unsigned mem instruction
    output logic                        dec_rd_write,       // rd write
    output logic [`REG_AW-1:0]          dec_rd_addr,        // rd address
    output logic [`REG_AW-1:0]          dec_rs1_addr,       // rs1 address
    output logic [`REG_AW-1:0]          dec_rs2_addr,       // rs2 address
    output logic [`XLEN-1:0]            dec_immediate,      // immediate value
    output logic                        dec_csr_write,      // csrrw/csrrwi
    output logic                        dec_csr_set,        // csrrs/csrrsi
    output logic                        dec_csr_clear,      // csrrc/csrrci
    output logic                        dec_csr_read,       // csr read
    output logic [11:0]                 dec_csr_addr,       // csr address
    output logic                        dec_mret,
    output logic                        dec_illegal_instr   // invalid instruction
);

    logic [1:0] rv32i_phase;
    logic [4:0] rv32i_opcode;
    logic [2:0] rv32i_funct3;
    logic [6:0] rv32i_funct7;

    logic [`XLEN-1:0] u_type_imm_val;
    logic [`XLEN-1:0] i_type_imm_val;
    logic [`XLEN-1:0] j_type_imm_val;
    logic [`XLEN-1:0] s_type_imm_val;
    logic [`XLEN-1:0] b_type_imm_val;
    logic [`XLEN-1:0] csr_type_imm_val;
    logic is_u_type_imm;
    logic is_i_type_imm;
    logic is_j_type_imm;
    logic is_s_type_imm;
    logic is_b_type_imm;
    logic is_csr_type_imm;

    logic phase3;
    logic is_lui;
    logic is_auipc;
    logic is_jal;
    logic is_jalr;
    logic is_itype;
    logic is_rtype;
    logic is_load;
    logic is_store;
    logic is_branch;


    logic rv32i_funct7_eq_0x0;
    logic rv32i_funct7_eq_0x20;
    logic rv32i_funct7_eq_0x18;

    // Instruction opcode
    logic is_add;
    logic is_sub;
    logic is_sll;
    logic is_slt;
    logic is_sltu;
    logic is_xor;
    logic is_srl;
    logic is_sra;
    logic is_or;
    logic is_and;
    logic is_system;
    logic is_csr;
    logic is_fence;

    // Load and Store
    logic load_is_unsigned;
    logic ls_is_half;
    logic ls_is_byte;
    logic ls_is_word;

    // Branch
    logic is_beq;
    logic is_bne;
    logic is_blt;
    logic is_bge;
    logic is_bltu;
    logic is_bgeu;
    logic branch_is_unsigned;

    // Invalid instruction
    logic invalid_opcode;
    logic invalid_jalr;
    logic invalid_bxx;
    logic invalid_load;
    logic invalid_store;
    logic invalid_itype_rtype;
    logic invalid_system;

    // [Optional] CSR
    logic is_csrrw; // csrrw/csrrwi
    logic is_csrrs; // csrrs/csrrsi
    logic is_csrrc; // csrrc/csrrci
    logic csr_use_imm;
    logic csrrw_read;
    logic csrrs_write;
    logic csrrc_write;

    // [Optional] RET (mret/sret)
    logic is_mret;

    // -------------------------------------------
    // Extract Each field from Instruction
    // -------------------------------------------
    assign rv32i_phase  = instruction[1:0];
    assign rv32i_opcode = instruction[6:2];
    assign rv32i_funct3 = instruction[14:12];
    assign rv32i_funct7 = instruction[31:25];

    assign u_type_imm_val = {instruction[31:12], 12'b0};
    assign i_type_imm_val = {{20{instruction[31]}}, instruction[31:20]};
    assign j_type_imm_val = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
    assign s_type_imm_val = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
    assign b_type_imm_val = {{20{instruction[31]}}, instruction[7],instruction[30:25],instruction[11:8], 1'b0};

    // -------------------------------------------
    // Decode the instruction
    // -------------------------------------------

    // RV32I Base Instruction Set (Not compressed)
    // Opcode decode
    assign phase3   = (rv32i_phase == 2'b11);
    assign is_lui   = phase3 & (rv32i_opcode == `RV32I_OPCODE_LUI);
    assign is_auipc = phase3 & (rv32i_opcode == `RV32I_OPCODE_AUIPC);
    assign is_jal   = phase3 & (rv32i_opcode == `RV32I_OPCODE_JAL);
    assign is_jalr  = phase3 & (rv32i_opcode == `RV32I_OPCODE_JALR);
    assign is_fence = phase3 & (rv32i_opcode == `RV32I_OPCODE_FENCE);
    assign is_load  = phase3 & (rv32i_opcode == `RV32I_OPCODE_LOAD);
    assign is_store = phase3 & (rv32i_opcode == `RV32I_OPCODE_STORE);
    assign is_branch = (rv32i_opcode == `RV32I_OPCODE_BRANCH);
    assign is_itype = phase3 & (rv32i_opcode == `RV32I_OPCODE_ITYPE);
    assign is_rtype = phase3 & (rv32i_opcode == `RV32I_OPCODE_RTYPE);
    assign is_system = (rv32i_opcode == `RV32I_OPCODE_SYSTEM);
    assign is_csr   = is_system;

    // Funct7
    assign rv32i_funct7_eq_0x0  = (rv32i_funct7 == 7'h00);
    assign rv32i_funct7_eq_0x20 = (rv32i_funct7 == 7'h20);
    assign rv32i_funct7_eq_0x18 = (rv32i_funct7 == 7'h18);

    // itype and rtype decode
    assign is_slt  = (rv32i_funct3 == `RV32I_FUNC3_SLT)  & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_sltu = (rv32i_funct3 == `RV32I_FUNC3_SLTU) & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_xor  = (rv32i_funct3 == `RV32I_FUNC3_XOR)  & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_or   = (rv32i_funct3 == `RV32I_FUNC3_OR)   & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_and  = (rv32i_funct3 == `RV32I_FUNC3_AND)  & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_add  = (rv32i_funct3 == `RV32I_FUNC3_ADD)  & (is_itype | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_srl  = (rv32i_funct3 == `RV32I_FUNC3_SRL)  & ((is_itype & & rv32i_funct7_eq_0x0) | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_sra  = (rv32i_funct3 == `RV32I_FUNC3_SRA)  & ((is_itype & & rv32i_funct7_eq_0x20)| (is_rtype & rv32i_funct7_eq_0x20));
    assign is_sll  = (rv32i_funct3 == `RV32I_FUNC3_SLL)  & ((is_itype & & rv32i_funct7_eq_0x0) | (is_rtype & rv32i_funct7_eq_0x0));
    assign is_sub  = (rv32i_funct3 == `RV32I_FUNC3_SUB)  & (is_rtype & rv32i_funct7_eq_0x20);

    // Load and Store decode
    assign load_is_unsigned = is_load & rv32i_funct3[2];
    assign ls_is_byte = (rv32i_funct3[1:0] == 2'h0);
    assign ls_is_half = (rv32i_funct3[1:0] == 2'h1);
    assign ls_is_word = (rv32i_funct3[1:0] == 2'h2);

    // Branch decode
    assign is_beq  = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BEQ);
    assign is_bne  = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BNE);
    assign is_blt  = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BLT);
    assign is_bge  = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BGE);
    assign is_bltu = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BLTU);
    assign is_bgeu = is_branch & (rv32i_funct3 == `RV32I_FUNC3_BGEU);
    assign branch_is_unsigned = is_branch & rv32i_funct3[1];

    generate
    if (SUPPORT_TRAP) begin: gen_ret
        assign is_mret = is_system & (rv32i_funct3 == 0) & rv32i_funct7_eq_0x18
                                   & (dec_rd_addr == 0)  & (dec_rs1_addr == 5'd0) & (dec_rs2_addr == 5'd2);
    end
    else begin: no_ret
        assign is_mret = 1'b0;
    end
    endgenerate

    // -------------------------------------------
    // Control signal generation
    // -------------------------------------------
    // register address
    assign dec_rs1_addr = instruction[19:15];
    assign dec_rs2_addr = instruction[24:20];
    assign dec_rd_addr  = instruction[11:7];

    // select pc
    assign dec_alu_src1_sel[`ALU_SRC1_PC] = is_jal | is_auipc | is_branch;
    assign dec_alu_src1_sel[`ALU_SRC1_ZERO] = is_lui;

    // select immediate value
    assign dec_alu_src2_sel_imm = dec_jump | is_branch | is_lui | is_auipc | is_itype | is_load | is_store |
                                  csr_use_imm;

    assign dec_alu_opcode[`ALU_OP_ADD] = is_add | is_branch | is_lui | is_auipc | is_load | is_store;
    assign dec_alu_opcode[`ALU_OP_SUB] = is_sub;
    assign dec_alu_opcode[`ALU_OP_SLL] = is_sll;
    assign dec_alu_opcode[`ALU_OP_SLT] = is_slt ;
    assign dec_alu_opcode[`ALU_OP_SLTU] = is_sltu;
    assign dec_alu_opcode[`ALU_OP_XOR] = is_xor;
    assign dec_alu_opcode[`ALU_OP_SRL] = is_srl;
    assign dec_alu_opcode[`ALU_OP_SRA] = is_sra;
    assign dec_alu_opcode[`ALU_OP_OR]  = is_or;
    assign dec_alu_opcode[`ALU_OP_AND] = is_and;

    assign dec_mem_opcode[`MEM_OP_BYTE] = ls_is_byte;
    assign dec_mem_opcode[`MEM_OP_HALF] = ls_is_half;
    assign dec_mem_opcode[`MEM_OP_WORD] = ls_is_word;

    assign dec_jump = is_jal | is_jalr;
    assign dec_rd_write = is_lui | is_auipc | dec_jump | is_itype | is_rtype | is_load | dec_csr_read;
    //assign dec_rs1_read = is_rtype | is_itype | is_jalr | is_branch | is_load | is_store | is_csr | is_csrrw | is_csrrs | is_csrrc;
    //assign dec_rs2_read = is_rtype | is_store | is_branch;
    assign dec_mem_read = is_load;
    assign dec_mem_write = is_store;

    assign dec_branch = is_branch;
    assign dec_branch_opcode[`BRANCH_OP_EQ] = ~rv32i_funct3[2];     // OP_EQ encode both beq and bne
    assign dec_branch_opcode[`BRANCH_OP_LT] = rv32i_funct3[2];      // OP_LT encode both blt(u) and bge(u)
    assign dec_branch_opcode[`BRANCH_OP_NEGATE] = rv32i_funct3[0];  // OP_NEGATE distinguish beq/bne and blt(u)/bge(u)

    assign dec_unsign = load_is_unsigned | branch_is_unsigned;

    assign dec_mret = is_mret;

    assign is_i_type_imm = is_jalr | is_itype | is_load;
    assign is_u_type_imm = is_lui | is_auipc;
    assign is_j_type_imm = is_jal;
    assign is_s_type_imm = is_store;
    assign is_b_type_imm = is_branch;
    assign is_csr_type_imm = csr_use_imm;
    assign dec_immediate = ({32{is_i_type_imm}} & i_type_imm_val) |
                           ({32{is_u_type_imm}} & u_type_imm_val) |
                           ({32{is_j_type_imm}} & j_type_imm_val) |
                           ({32{is_s_type_imm}} & s_type_imm_val) |
                           ({32{is_b_type_imm}} & b_type_imm_val) |
                           ({32{is_csr_type_imm}} & csr_type_imm_val);

    // Note: doing nothing for fence instruction at this point because our execution is in order.

    // -------------------------------------------
    // Optional Features
    // -------------------------------------------

    // CSR
    generate
    if (SUPPORT_ZICSR) begin: gen_ISA_ZICSR
        assign csrrw_read  = |dec_rd_addr;  // csrrw/csrrwi should not read CSR if rd = x0
        assign csrrs_write = |dec_rs1_addr; // csrrs/csrrsi should not write CSR if rs1 = x0 (uimm = 0)
        assign csrrc_write = csrrs_write;   // same condition as csrrs/csrrsi

        assign is_csrrw = is_csr & (rv32i_funct3[1:0] == 2'b01);
        assign is_csrrs = is_csr & (rv32i_funct3[1:0] == 2'b10);
        assign is_csrrc = is_csr & (rv32i_funct3[1:0] == 2'b11);
        assign dec_csr_write = is_csrrw;
        assign dec_csr_set   = is_csrrs & csrrs_write;
        assign dec_csr_clear = is_csrrc & csrrc_write;
        assign dec_csr_read  = is_csrrw & csrrw_read | is_csrrs | is_csrrc;
        assign dec_csr_addr  = instruction[31:20];
        assign csr_use_imm   = is_csr & rv32i_funct3[2];
        assign csr_type_imm_val = {{`XLEN-5{1'b0}}, dec_rs1_addr};
    end
    else begin: no_ISA_ZICSR
        assign is_csrrw = 1'b0;
        assign is_csrrs = 1'b0;
        assign is_csrrc = 1'b0;
        assign csr_use_imm = 1'b0;
        assign csrrw_read = 1'b0;
        assign csrrs_write = 1'b0;
        assign csrrc_write = 1'b0;
        assign dec_csr_read  = 1'b0;
        assign dec_csr_write = 1'b0;
        assign dec_csr_clear = 1'b0;
        assign dec_csr_set   = 1'b0;
        assign dec_csr_addr  = 12'b0;
        assign csr_type_imm_val = `XLEN'h0;
    end
    endgenerate

    // Illegal Instruction
    generate
    if (SUPPORT_TRAP) begin: gen_exception
        assign invalid_opcode = ~(is_lui | is_auipc | is_rtype | is_jalr | is_itype  |
                                  is_jal | is_fence | is_store | is_load | is_branch |
                                  ((SUPPORT_ZICSR | SUPPORT_TRAP) & is_system));
        assign invalid_jalr = is_jalr & (|rv32i_funct3); // For JALR funct = 0
        assign invalid_bxx = is_branch & (rv32i_funct3[2:1] == 2'b01);
        assign invalid_load = is_load & ((rv32i_funct3[2:1] == 2'b11 ) | (rv32i_funct3[1:0]==2'b11));
        assign invalid_store = is_store & (rv32i_funct3[2] | (rv32i_funct3[1:0]==2'b11));
        assign invalid_itype_rtype = (is_itype | is_rtype) &
                                    ~(is_slt | is_sltu | is_xor | is_or  | is_and |
                                      is_add | is_srl  | is_sra | is_sll | is_sub);
        assign invalid_system = is_system & (
                                            ~is_mret &
                                            SUPPORT_ZICSR & ~(is_csrrw | is_csrrs | is_csrrc)
                                            );
        assign dec_illegal_instr = invalid_opcode | invalid_jalr   | invalid_bxx | invalid_load |
                                    invalid_store | invalid_system | invalid_itype_rtype;
    end
    else begin: no_exception
        assign invalid_opcode = 1'b0;
        assign invalid_jalr = 1'b0;
        assign invalid_bxx = 1'b0;
        assign invalid_load = 1'b0;
        assign invalid_store = 1'b0;
        assign invalid_itype_rtype = 1'b0;
        assign invalid_system = 1'b0;
        assign dec_illegal_instr = 1'b0;
    end
    endgenerate

endmodule
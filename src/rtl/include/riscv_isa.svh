/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/14/2023
 *
 * ------------------------------------------------------------------------------------------------
 * RISCV ISA
 * ------------------------------------------------------------------------------------------------
 */

// RV32I Instruction Set
`define RV32I_OPCODE_LUI       5'b01101
`define RV32I_OPCODE_AUIPC     5'b00101
`define RV32I_OPCODE_JAL       5'b11011
`define RV32I_OPCODE_JALR      5'b11001
`define RV32I_OPCODE_ITYPE     5'b00100
`define RV32I_OPCODE_RTYPE     5'b01100
`define RV32I_OPCODE_BRANCH    5'b11000
`define RV32I_OPCODE_LOAD      5'b00000
`define RV32I_OPCODE_STORE     5'b01000

`define RV32I_FUNC3_ADD     3'b000
`define RV32I_FUNC3_SUB     3'b000
`define RV32I_FUNC3_SLL     3'b001
`define RV32I_FUNC3_SLT     3'b010
`define RV32I_FUNC3_SLTU    3'b011
`define RV32I_FUNC3_XOR     3'b100
`define RV32I_FUNC3_SRL     3'b101
`define RV32I_FUNC3_SRA     3'b101
`define RV32I_FUNC3_OR      3'b110
`define RV32I_FUNC3_AND     3'b111

`define RV32I_FUNC3_BEQ     3'b000
`define RV32I_FUNC3_BNE     3'b001
`define RV32I_FUNC3_BLT     3'b100
`define RV32I_FUNC3_BGE     3'b101
`define RV32I_FUNC3_BLTU    3'b110
`define RV32I_FUNC3_BGEU    3'b111
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
`define RV32I_OPCODE_CSR       5'b11100
`define RV32I_OPCODE_FENCE     5'b00011

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

`define RV32I_FUNC3_CSRRW   3'b001
`define RV32I_FUNC3_CSRRS   3'b010
`define RV32I_FUNC3_CSRRC   3'b011
`define RV32I_FUNC3_CSRRWI  3'b101
`define RV32I_FUNC3_CSRRSI  3'b110
`define RV32I_FUNC3_CSRRCI  3'b111

// CSR address
`define MSTATUS             12'h300
`define MIE                 12'h304
`define MTVEC               12'h305

`define MSCRATCH            12'h340
`define MEPC                12'h341
`define MCAUSE              12'h342
`define MTVAL               12'h343
`define MIP                 12'h344

// Exception code
`define INSTR_ADDR_MISALIGNED   4'd0
`define ILLEGAL_INSTRUCTION     4'd2
`define LOAD_ADDR_MISALIGNED    4'd4
`define STORE_ADDR_MISALIGNED   4'd6

// Interrupt code
`define M_SOFTWARE_INTERRUPT    4'd3
`define M_TIMER_INTERRUPT       4'd7
`define M_EXTERNAL_INTERRUPT    4'd11

/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/13/2023
 *
 * ------------------------------------------------------------------------------------------------
 * Include File for Core logic
 * ------------------------------------------------------------------------------------------------
 */

`ifndef __RV32COREF_CORE__
`define __RV32COREF_CORE__

// ALU Opcode
`define ALU_OP_ADD      0
`define ALU_OP_SUB      1
`define ALU_OP_SLL      2
`define ALU_OP_SLT      3
`define ALU_OP_SLTU     4
`define ALU_OP_XOR      5
`define ALU_OP_SRL      6
`define ALU_OP_SRA      7
`define ALU_OP_OR       8
`define ALU_OP_AND      9
`define ALU_OP_WIDTH    10

// Memory Opcode
`define MEM_OP_BYTE     0
`define MEM_OP_HALF     1
`define MEM_OP_WORD     2
`define MEM_OP_WIDTH    3

// Branch Opcode
`define BRANCH_OP_EQ    0
`define BRANCH_OP_LT    1
`define BRANCH_OP_NEGATE 2
`define BRANCH_OP_WIDTH 3

// ALU SRC 1 selection
`define ALU_SRC1_PC     0
`define ALU_SRC1_ZERO   1
`define ALU_SRC1_WIDTH  2

// Multiple/Divide
`define MUL_OP_WIDTH    2
`define DIV_OP_WIDTH    2

`endif
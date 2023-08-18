/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/16/2023
 *
 * ------------------------------------------------------------------------------------------------
 * CPU Core top level
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"
`include "core.svh"

module core (
    input  logic                clk,
    input  logic                rst_b,
    // Instruction RAM Access
    output logic                iram_req,
    output logic                iram_write,
    output logic [`XLEN/8-1:0]  iram_wstrb,
    output logic [`XLEN-1:0]    iram_addr,
    output logic [`XLEN-1:0]    iram_wdata,
    output logic                iram_addr_ok,
    input  logic                iram_data_ok,
    input  logic [`XLEN-1:0]    iram_rdata
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // IF <--> ID
    logic                           id_pipe_valid;
    logic                           id_pipe_ready;
    logic [`XLEN-1:0]               id_pc;
    logic [`XLEN-1:0]               id_instruction;

    // ID <--> EX
    logic                           ex_pipe_valid;
    logic                           ex_pipe_ready;
    logic [`XLEN-1:0]               ex_pc;
    logic [`XLEN-1:0]               ex_instruction;
    logic [`ALU_OP_WIDTH-1:0]       ex_alu_opcode;
    logic                           ex_alu_src1_sel_pc;
    logic                           ex_alu_src2_sel_imm;
    logic                           ex_branch;
    logic [`BRANCH_OP_WIDTH-1:0]    ex_branch_opcode;
    logic                           ex_jump;
    logic [`XLEN-1:0]               ex_rs1_rdata;
    logic [`XLEN-1:0]               ex_rs2_rdata;
    logic                           ex_rd_write;
    logic [`REG_AW-1:0]             ex_rd_addr;
    logic                           ex_mem_read;
    logic                           ex_mem_write;
    logic [`MEM_OP_WIDTH-1:0]       ex_mem_opcode;
    logic                           ex_unsign;
    logic [`XLEN-1:0]               ex_immediate;

    // WB <--> ID
    logic                           wb_rd_write;
    logic [`XLEN-1:0]               wb_rd_wdata;
    logic [`REG_AW-1:0]             wb_rd_addr;

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    IF u_if(.*);

    ID u_id(.*);

endmodule
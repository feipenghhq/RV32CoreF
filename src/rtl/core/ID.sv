/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/16/2023
 *
 * ------------------------------------------------------------------------------------------------
 * ID: Instruction Decode Stage
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"
`include "core.svh"

module ID (
    input  logic                        clk,
    input  logic                        rst_b,
    // IF <--> ID Pipeline
    output logic                        id_pipe_ready,
    input  logic                        id_pipe_valid,
    input  logic [`XLEN-1:0]            id_pc,
    input  logic [`XLEN-1:0]            id_instruction,
    // ID <--> EX Pipeline
    input  logic                        ex_pipe_ready,
    output logic                        ex_pipe_valid,
    output logic [`XLEN-1:0]            ex_pc,
    output logic [`XLEN-1:0]            ex_instruction,
    output logic [`ALU_OP_WIDTH-1:0]    ex_alu_opcode,
    output logic                        ex_alu_src1_sel,
    output logic                        ex_alu_src2_sel,
    output logic                        ex_branch,
    output logic [`BRANCH_OP_WIDTH-1:0] ex_branch_opcode,
    output logic                        ex_jump,
    output logic [`XLEN-1:0]            ex_rs1_rdata,
    output logic [`XLEN-1:0]            ex_rs2_rdata,
    output logic                        ex_rd_write,
    output logic [`REG_AW-1:0]          ex_rd_addr,
    output logic                        ex_mem_read,
    output logic                        ex_mem_write,
    output logic [`MEM_OP_WIDTH-1:0]    ex_mem_opcode,
    output logic                        ex_unsign,
    output logic [`XLEN-1:0]            ex_immediate,
    // WB <--> ID
    input  logic                        wb_rd_write,
    input  logic [`XLEN-1:0]            wb_rd_wdata,
    input  logic [`REG_AW-1:0]          wb_rd_addr
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic id_pipe_done;
    logic id_pipe_req;

    // From/to regfile
    logic [`REG_AW-1:0]             rs1_addr;
    logic [`XLEN-1:0]               rs1_rdata;
    logic [`REG_AW-1:0]             rs2_addr;
    logic [`XLEN-1:0]               rs2_rdata;
    logic                           rd_write;
    logic [`REG_AW-1:0]             rd_addr;

    // From/to decoder
    logic [`ALU_OP_WIDTH-1:0]       alu_opcode;
    logic                           alu_src1_sel;
    logic                           alu_src2_sel;
    logic                           branch;
    logic [`BRANCH_OP_WIDTH-1:0]    branch_opcode;
    logic                           jump;
    logic                           rs1_read;
    logic                           rs2_read;
    logic                           mem_read;
    logic                           mem_write;
    logic [`MEM_OP_WIDTH-1:0]       mem_opcode;
    logic                           unsign;
    logic [`XLEN-1:0]               immediate;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign id_pipe_done = 1'b1;
    assign id_pipe_ready = ex_pipe_ready & id_pipe_done;
    assign id_pipe_req = id_pipe_done & id_pipe_valid;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (rst_b)         ex_pipe_valid <= 1'b0;
        else if (ex_pipe_ready) ex_pipe_valid <= id_pipe_req;
    end

    always @(posedge clk) begin
        ex_pc <= id_pc;
        ex_instruction <= id_instruction;
        ex_alu_opcode <= alu_opcode;
        ex_alu_src1_sel <= alu_src1_sel;
        ex_alu_src2_sel <= alu_src2_sel;
        ex_branch <= branch;
        ex_branch_opcode <= branch_opcode;
        ex_jump <= jump;
        ex_rs1_rdata <= rs1_rdata;
        ex_rs1_rdata <= rs1_rdata;
        ex_rd_write <= rd_write;
        ex_rd_addr <= rd_addr;
        ex_mem_read <= mem_read;
        ex_mem_write <= mem_write;
        ex_mem_opcode <= mem_opcode;
        ex_unsign <= unsign;
        ex_immediate <= immediate;
    end

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------
    regfile u_regfile(
        .rd_write(wb_rd_write),
        .rd_wdata(wb_rd_wdata),
        .rd_addr(wb_rd_addr),
        .*);

    decoder u_decoder(
        .instruction(id_instruction),
        .*);

endmodule
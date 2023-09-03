/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/16/2023
 *
 * ------------------------------------------------------------------------------------------------
 * IF: Instruction Fetch Stage
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"

module IF (
    input  logic                clk,
    input  logic                rst_b,
    // IF <--> ID Pipeline
    output logic                id_pipe_valid,
    input  logic                id_pipe_ready,
    input  logic                id_pipe_flush,
    output logic [`XLEN-1:0]    id_pipe_pc,
    output logic [`XLEN-1:0]    id_pipe_instruction,
    // EX --> IF
    input  logic                ex_branch,       // jump and taken branch
    input  logic [`XLEN-1:0]    ex_branch_pc,    // target pc
    // Instruction RAM Access
    output logic                iram_req,
    output logic                iram_write,
    output logic [`XLEN/8-1:0]  iram_wstrb,
    output logic [`XLEN-1:0]    iram_addr,
    output logic [`XLEN-1:0]    iram_wdata,
    input  logic                iram_addr_ok,
    input  logic                iram_data_ok,
    input  logic [`XLEN-1:0]    iram_rdata
);
    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic if_pipe_done;
    logic if_pipe_req;
    logic if_valid;

    // From IFU
    logic [`XLEN-1:0] pc_val;
    logic [`XLEN-1:0] instruction;
    logic             instr_valid;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign if_valid = ~id_pipe_flush;
    assign if_pipe_done = instr_valid; // FIXME: need to consider iram_addr_ok and iram_data_ok
    assign if_pipe_req = if_pipe_done & if_valid;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        id_pipe_valid <= 1'b0;
        else if (id_pipe_ready) id_pipe_valid <= if_pipe_req & ~id_pipe_flush;
    end

    always @(posedge clk) begin
        id_pipe_pc <= pc_val;
        id_pipe_instruction <= instruction;
    end

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    ifu u_ifu(
        .branch(ex_branch),
        .branch_pc(ex_branch_pc),
        .*
        );

endmodule
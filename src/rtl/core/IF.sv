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
    output logic [`XLEN-1:0]    id_pc,
    output logic [`XLEN-1:0]    id_instruction,
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

    // Pipeline Control
    logic if_pipe_done;
    logic if_pipe_req;

    // From IFU
    logic [`XLEN-1:0] pc_val;
    logic [`XLEN-1:0] instruction;
    logic             instr_valid;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign if_pipe_done = instr_valid;
    assign if_pipe_req = if_pipe_done;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (rst_b)         id_pipe_valid <= 1'b0;
        else if (id_pipe_ready) id_pipe_valid <= if_pipe_req;
    end

    always @(posedge clk) begin
        id_pc <= pc_val;
        id_instruction <= instruction;
    end

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    ifu u_ifu(.*);

endmodule
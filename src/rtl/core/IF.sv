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

    // PC logic
    logic [`XLEN-1:0] pc;
    logic [`XLEN-1:0] next_pc;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign if_valid = ~id_pipe_flush;
    assign if_pipe_done = iram_data_ok; // FIXME: need to consider iram_addr_ok and iram_data_ok being zero cases
    assign if_pipe_req = if_pipe_done & if_valid;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        id_pipe_valid <= 1'b0;
        else if (id_pipe_ready) id_pipe_valid <= if_pipe_req & ~id_pipe_flush;
    end

    always @(posedge clk) begin
        if (id_pipe_ready) begin
            id_pipe_pc <= pc_val;
            id_pipe_instruction <= instruction;
        end
    end

    // -------------------------------------------
    // Instruction RAM logic
    // -------------------------------------------
    // 1. always read the instruction ram
    // 2. Assume that the ram is synchronouse ram and data come back at the next clock cycle
    // 3. We introduce a "Pre IF" stage where we update the PC. We send the read request on
    // the "Pre IF" stage and the address is next_pc so when data comes back it is in IF stage
    assign iram_req = 1'b1;
    assign iram_write = 1'b0;
    assign iram_wstrb = '0;
    assign iram_addr = next_pc;
    assign iram_wdata = '0;

    // -------------------------------------------
    // PC logic
    // -------------------------------------------

    assign pc_val = pc;
    assign next_pc = ex_branch ? ex_branch_pc : pc + `XLEN'h4;

    always @(posedge clk) begin
        if (!rst_b) begin
            pc <= `PC_RESET_ADDR - 4; // because we use next_pc for ram address, we need to minus 4 here
        end
        else if (id_pipe_ready) begin // only update the pc when the instruction is ready to move to next stage
            pc <= next_pc;
        end
    end

    // -------------------------------------------
    // Instruction logic
    // -------------------------------------------
    assign instruction = iram_rdata;

endmodule
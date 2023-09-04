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
    logic preif_done;
    logic preif_req;
    logic preif_valid;
    logic preif_pipe_valid;

    logic if_done;
    logic if_req;
    logic if_valid;
    logic if_pipe_ready;

    // From IFU
    logic [`XLEN-1:0] pc_val;
    logic [`XLEN-1:0] instruction;
    logic             instr_valid;

    // PC logic
    logic [`XLEN-1:0] pc;
    logic [`XLEN-1:0] next_pc;

    // Instruction logic
    logic [`XLEN-1:0] if_instruction;       // a register to hold the instruction in IF stage in case of
                                            // id is not ready
    logic             if_instruction_valid;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign preif_done = iram_req & iram_addr_ok;
    assign preif_req = preif_done;
    assign preif_valid = preif_req; // at the current implementation, preif won't be flushed because we always want to
                                    // read the next instruction even if we flush other stages.

    always @(posedge clk) begin
        if (!rst_b) preif_pipe_valid <= 1'b0;
        else if (if_pipe_ready) preif_pipe_valid <= preif_valid;
    end

    assign if_valid = preif_pipe_valid & ~id_pipe_flush;
    assign if_done = iram_data_ok | if_instruction_valid;
    assign if_req = if_done & if_valid;
    assign if_pipe_ready = ~if_valid | if_req & id_pipe_ready;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        id_pipe_valid <= 1'b0;
        else if (id_pipe_ready) id_pipe_valid <= if_req & ~id_pipe_flush;
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
    assign iram_req = if_pipe_ready;     // we only read the memory when IF stage is ready
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

    // If IF is request to send data to ID but ID is not ready, put the data in if_instruction and set the valid bit
    // If ID is ready while if_instruction is valid, clear the valid bit and select the data from if_instruction
    always @(posedge clk) begin
        if (!rst_b) begin
            if_instruction_valid <= 1'b0;
        end
        else begin
            if (if_req && !id_pipe_ready) if_instruction_valid <= 1'b1;
            else if (id_pipe_ready) if_instruction_valid <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (if_req && !id_pipe_ready) if_instruction <= iram_rdata;
    end

    assign instruction = if_instruction_valid ? if_instruction : iram_rdata;

endmodule
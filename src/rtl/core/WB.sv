/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/18/2023
 *
 * ------------------------------------------------------------------------------------------------
 * WB: Write back stage
 * ------------------------------------------------------------------------------------------------
 */

`include "core.svh"
`include "config.svh"

module WB (
    input  logic                        clk,
    input  logic                        rst_b,
    // MEM <--> WB Pipeline
    output logic                        wb_pipe_ready,
    output logic                        wb_pipe_flush,
    input  logic                        wb_pipe_valid,
    input  logic [`XLEN-1:0]            wb_pipe_pc,
    input  logic [`XLEN-1:0]            wb_pipe_instruction,
    input  logic                        wb_pipe_rd_write,
    input  logic [`REG_AW-1:0]          wb_pipe_rd_addr,
    input  logic [`XLEN-1:0]            wb_pipe_rd_data,
    // To register writeback
    output logic                        wb_rd_write,
    output logic [`REG_AW-1:0]          wb_rd_addr,
    output logic [`XLEN-1:0]            wb_rd_wdata
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic wb_done;
    logic wb_req;
    logic wb_valid;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign wb_valid = wb_pipe_valid;
    assign wb_done = 1'b1;
    assign wb_pipe_ready = wb_done;
    assign wb_req = wb_done & wb_valid;
    assign wb_pipe_flush = 1'b0;

    // register write logic
    assign wb_rd_write = wb_pipe_rd_write & wb_pipe_valid;
    assign wb_rd_addr = wb_pipe_rd_addr;
    assign wb_rd_wdata = wb_pipe_rd_data;

endmodule
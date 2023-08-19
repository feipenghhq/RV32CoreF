/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/18/2023
 *
 * ------------------------------------------------------------------------------------------------
 * MEM: memory Stage
 * ------------------------------------------------------------------------------------------------
 */

`include "core.svh"
`include "config.svh"

module MEM (
    input  logic                        clk,
    input  logic                        rst_b,
    // EX <--> MEM Pipeline
    output logic                        mem_pipe_ready,
    output logic                        mem_pipe_flush,
    input  logic                        mem_pipe_valid,
    input  logic [`XLEN-1:0]            mem_pipe_pc,
    input  logic [`XLEN-1:0]            mem_pipe_instruction,
    input  logic                        mem_pipe_mem_read,
    input  logic [`MEM_OP_WIDTH-1:0]    mem_pipe_mem_opcode,
    input  logic                        mem_pipe_unsign,
    input  logic                        mem_pipe_rd_write,
    input  logic [`REG_AW-1:0]          mem_pipe_rd_addr,
    input  logic [`XLEN-1:0]            mem_pipe_alu_result,
    // MEM <--> WB Pipeline
    input  logic                        wb_pipe_ready,
    input  logic                        wb_pipe_flush,
    output logic                        wb_pipe_valid,
    output logic [`XLEN-1:0]            wb_pipe_pc,
    output logic [`XLEN-1:0]            wb_pipe_instruction,
    output logic                        wb_pipe_rd_write,
    output logic [`REG_AW-1:0]          wb_pipe_rd_addr,
    output logic [`XLEN-1:0]            wb_pipe_rd_data,
    // MEM to other stage
    output logic                        mem_rd_write,
    output logic [`REG_AW-1:0]          mem_rd_addr,
    output logic [`XLEN-1:0]            mem_rd_wdata,
    // Data RAM Access
    input  logic                        dram_data_ok,
    input  logic [`XLEN-1:0]            dram_rdata
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic mem_pipe_done;
    logic mem_pipe_req;
    logic mem_valid;

    // Memory Read Control
    logic               load_done;
    logic [`XLEN-1:0]   load_data;
    logic [7:0]         lb_data;
    logic [15:0]        lh_data;
    logic [`XLEN-1:0]   lb_ext_data;
    logic [`XLEN-1:0]   lbu_ext_data;
    logic [`XLEN-1:0]   lh_ext_data;
    logic [`XLEN-1:0]   lhu_ext_data;
    logic               is_lb;
    logic               is_lbu;
    logic               is_lh;
    logic               is_lhu;
    logic               is_lw;

    // register write back
    logic [`XLEN-1:0]   rd_data;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign mem_valid = mem_pipe_valid & ~wb_pipe_flush;
    assign mem_pipe_done = load_done;
    assign mem_pipe_ready = wb_pipe_ready & mem_pipe_done;
    assign mem_pipe_req = mem_pipe_done & mem_valid;
    assign mem_pipe_flush = wb_pipe_flush;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (rst_b)         wb_pipe_valid <= 1'b0;
        else if (wb_pipe_ready) wb_pipe_valid <= mem_pipe_req;
    end

    always @(posedge clk) begin
        wb_pipe_pc <= mem_pipe_pc;
        wb_pipe_instruction <= mem_pipe_instruction;
        wb_pipe_rd_write <= mem_pipe_rd_write;
        wb_pipe_rd_addr <= mem_pipe_rd_addr;
        wb_pipe_rd_data <= rd_data;
    end

    // --------------------------------------
    // Memory Read Control
    // --------------------------------------

    assign is_lb  = mem_pipe_mem_opcode[`MEM_OP_BYTE] & ~mem_pipe_unsign;
    assign is_lbu = mem_pipe_mem_opcode[`MEM_OP_BYTE] & mem_pipe_unsign;
    assign is_lh  = mem_pipe_mem_opcode[`MEM_OP_HALF] & ~mem_pipe_unsign;
    assign is_lhu = mem_pipe_mem_opcode[`MEM_OP_HALF] & mem_pipe_unsign;
    assign is_lw  = mem_pipe_mem_opcode[`MEM_OP_WORD];

    assign lb_ext_data  = {{(`XLEN-8){lb_data[7]}},  lb_data};
    assign lbu_ext_data = {{(`XLEN-8){1'b0}},        lb_data};
    assign lh_ext_data  = {{(`XLEN-16){lh_data[15]}},lh_data};
    assign lhu_ext_data = {{(`XLEN-16){1'b0}},       lh_data};

    assign lb_data = ({8{mem_pipe_rd_addr[1:0] == 0}} & dram_rdata[ 7: 0]) |
                     ({8{mem_pipe_rd_addr[1:0] == 1}} & dram_rdata[15: 8]) |
                     ({8{mem_pipe_rd_addr[1:0] == 2}} & dram_rdata[23:16]) |
                     ({8{mem_pipe_rd_addr[1:0] == 3}} & dram_rdata[31:24]);

    assign lh_data = mem_pipe_rd_addr[1] ? dram_rdata[`XLEN-1:16] : dram_rdata[15:0];

    assign load_data = ({`XLEN{is_lb}}  & lb_ext_data)  |
                       ({`XLEN{is_lbu}} & lbu_ext_data) |
                       ({`XLEN{is_lh}}  & lh_ext_data)  |
                       ({`XLEN{is_lhu}} & lhu_ext_data) |
                       ({`XLEN{is_lw}}  & dram_rdata) ;

    assign load_done = mem_valid & mem_pipe_mem_read & dram_data_ok;

    // --------------------------------------
    // Registr write back data selection
    // --------------------------------------
    assign rd_data = mem_pipe_mem_read ? load_data : mem_pipe_alu_result;

    // --------------------------------------
    // Forward logic to ID stage
    // --------------------------------------
    // no need to check if ex_valid, if ex is not valid, then the dependency by nature would be failed.
    assign mem_rd_write = mem_pipe_rd_write;
    assign mem_rd_addr = mem_pipe_rd_addr;
    assign mem_rd_wdata = rd_data;

endmodule
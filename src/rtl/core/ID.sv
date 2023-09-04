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
    output logic                        id_pipe_flush,
    input  logic                        id_pipe_valid,
    input  logic [`XLEN-1:0]            id_pipe_pc,
    input  logic [`XLEN-1:0]            id_pipe_instruction,
    // ID <--> EX Pipeline
    input  logic                        ex_pipe_ready,
    input  logic                        ex_pipe_flush,
    output logic                        ex_pipe_valid,
    output logic [`XLEN-1:0]            ex_pipe_pc,
    output logic [`XLEN-1:0]            ex_pipe_instruction,
    output logic [`ALU_OP_WIDTH-1:0]    ex_pipe_alu_opcode,
    output logic [`ALU_SRC1_WIDTH-1:0]  ex_pipe_alu_src1_sel,
    output logic                        ex_pipe_alu_src2_sel_imm,
    output logic                        ex_pipe_branch,
    output logic [`BRANCH_OP_WIDTH-1:0] ex_pipe_branch_opcode,
    output logic                        ex_pipe_jump,
    output logic [`XLEN-1:0]            ex_pipe_rs1_rdata,
    output logic [`XLEN-1:0]            ex_pipe_rs2_rdata,
    output logic                        ex_pipe_rd_write,
    output logic [`REG_AW-1:0]          ex_pipe_rd_addr,
    output logic                        ex_pipe_mem_read,
    output logic                        ex_pipe_mem_write,
    output logic [`MEM_OP_WIDTH-1:0]    ex_pipe_mem_opcode,
    output logic                        ex_pipe_unsign,
    output logic [`XLEN-1:0]            ex_pipe_immediate,
    // WB --> ID
    input  logic                        wb_rd_write,
    input  logic [`XLEN-1:0]            wb_rd_wdata,
    input  logic [`REG_AW-1:0]          wb_rd_addr,
    // MEM --> ID
    input  logic                        mem_rd_write,
    input  logic [`REG_AW-1:0]          mem_rd_addr,
    input  logic [`XLEN-1:0]            mem_rd_wdata,
    // EX --> ID
    input  logic                        ex_rd_write,
    input  logic [`REG_AW-1:0]          ex_rd_addr,
    input  logic [`XLEN-1:0]            ex_rd_wdata
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic id_valid;
    logic id_done;
    logic id_req;

    // From/to regfile
    logic [`REG_AW-1:0]             dec_rs1_addr;
    logic [`REG_AW-1:0]             dec_rs2_addr;
    logic                           dec_rd_write;
    logic [`REG_AW-1:0]             dec_rd_addr;
    logic [`XLEN-1:0]               rs1_rdata;
    logic [`XLEN-1:0]               rs2_rdata;
    logic [`XLEN-1:0]               rs1_rdata_forwarded;
    logic [`XLEN-1:0]               rs2_rdata_forwarded;

    // From/to decoder
    logic [`ALU_OP_WIDTH-1:0]       dec_alu_opcode;
    logic [`ALU_SRC1_WIDTH-1:0]     dec_alu_src1_sel;
    logic                           dec_alu_src2_sel_imm;
    logic                           dec_branch;
    logic [`BRANCH_OP_WIDTH-1:0]    dec_branch_opcode;
    logic                           dec_jump;
    logic                           dec_rs1_read;
    logic                           dec_rs2_read;
    logic                           dec_mem_read;
    logic                           dec_mem_write;
    logic [`MEM_OP_WIDTH-1:0]       dec_mem_opcode;
    logic                           dec_unsign;
    logic [`XLEN-1:0]               dec_immediate;

    // Forward logic
    logic                           rs1_match_ex;
    logic                           rs1_match_mem;
    logic                           rs1_match_wb;
    logic                           rs2_match_ex;
    logic                           rs2_match_mem;
    logic                           rs2_match_wb;

    // Stall logic
    logic                           id_depends_on_load;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign id_valid = id_pipe_valid & ~ex_pipe_flush;
    assign id_done  = ~id_depends_on_load;
    assign id_req   = id_done & id_valid;

    assign id_pipe_ready = ~id_valid | id_req & ex_pipe_ready;
    assign id_pipe_flush = ex_pipe_flush;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        ex_pipe_valid <= 1'b0;
        else if (ex_pipe_ready) ex_pipe_valid <= id_req;
    end

    always @(posedge clk) begin
        if (ex_pipe_ready) begin
            ex_pipe_pc <= id_pipe_pc;
            ex_pipe_instruction <= id_pipe_instruction;
            ex_pipe_alu_opcode <= dec_alu_opcode;
            ex_pipe_alu_src1_sel <= dec_alu_src1_sel;
            ex_pipe_alu_src2_sel_imm <= dec_alu_src2_sel_imm;
            ex_pipe_branch <= dec_branch;
            ex_pipe_branch_opcode <= dec_branch_opcode;
            ex_pipe_jump <= dec_jump;
            ex_pipe_rs1_rdata <= rs1_rdata_forwarded;
            ex_pipe_rs2_rdata <= rs2_rdata_forwarded;
            ex_pipe_rd_write <= dec_rd_write;
            ex_pipe_rd_addr <= dec_rd_addr;
            ex_pipe_mem_read <= dec_mem_read;
            ex_pipe_mem_write <= dec_mem_write;
            ex_pipe_mem_opcode <= dec_mem_opcode;
            ex_pipe_unsign <= dec_unsign;
            ex_pipe_immediate <= dec_immediate;
        end
    end

    // --------------------------------------
    // Forwarding Logic
    // --------------------------------------
    assign rs1_match_ex  = (dec_rs1_addr == ex_rd_addr)  & ex_rd_write;
    assign rs1_match_mem = (dec_rs1_addr == mem_rd_addr) & mem_rd_write;
    assign rs1_match_wb  = (dec_rs1_addr == wb_rd_addr)  & wb_rd_write;
    assign rs2_match_ex  = (dec_rs2_addr == ex_rd_addr)  & ex_rd_write;
    assign rs2_match_mem = (dec_rs2_addr == mem_rd_addr) & mem_rd_write;
    assign rs2_match_wb  = (dec_rs2_addr == wb_rd_addr)  & wb_rd_write;

    assign rs1_rdata_forwarded = (dec_rs1_addr == 0) ? 0 :
                                 rs1_match_ex ? ex_rd_wdata :
                                 rs1_match_mem ? mem_rd_wdata :
                                 rs1_match_wb ? wb_rd_wdata  :
                                 rs1_rdata;

    assign rs2_rdata_forwarded = (dec_rs2_addr == 0) ? 0 :
                                 rs2_match_ex ? ex_rd_wdata :
                                 rs2_match_mem ? mem_rd_wdata :
                                 rs2_match_wb ? wb_rd_wdata :
                                 rs2_rdata;

    // --------------------------------------
    // Stall Logic
    // --------------------------------------
    // We need to stall if there is data dependencies on load instructions
    // because the load data is ready in MEM stage.
    // If load instruction is waiting in MEM stage, then the MEM stage will
    // backpressure all the previous stage so no need to have extral stall
    // logic for this situation.
    assign id_depends_on_load = ex_pipe_mem_read & ex_pipe_valid & (rs1_match_ex | rs2_match_ex);

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------
    regfile #(.R0_ZERO(0))
    u_regfile(
        .rs1_addr(dec_rs1_addr),
        .rs2_addr(dec_rs2_addr),
        .rd_write(wb_rd_write),
        .rd_addr(wb_rd_addr),
        .rd_wdata(wb_rd_wdata),
        .*);

    decoder u_decoder(
        .instruction(id_pipe_instruction),
        .*);

endmodule
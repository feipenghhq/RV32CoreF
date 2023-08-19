/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/16/2023
 *
 * ------------------------------------------------------------------------------------------------
 * EX: Execution Stage
 * ------------------------------------------------------------------------------------------------
 */

`include "core.svh"
`include "config.svh"

module EX (
    input  logic                        clk,
    input  logic                        rst_b,
    // ID <--> EX Pipeline
    output logic                        ex_pipe_ready,
    output logic                        ex_pipe_flush,
    input  logic                        ex_pipe_valid,
    input  logic [`XLEN-1:0]            ex_pipe_pc,
    input  logic [`XLEN-1:0]            ex_pipe_instruction,
    input  logic [`ALU_OP_WIDTH-1:0]    ex_pipe_alu_opcode,
    input  logic                        ex_pipe_alu_src1_sel_pc,
    input  logic                        ex_pipe_alu_src2_sel_imm,
    input  logic                        ex_pipe_branch,
    input  logic [`BRANCH_OP_WIDTH-1:0] ex_pipe_branch_opcode,
    input  logic                        ex_pipe_jump,
    input  logic [`XLEN-1:0]            ex_pipe_rs1_rdata,
    input  logic [`XLEN-1:0]            ex_pipe_rs2_rdata,
    input  logic                        ex_pipe_rd_write,
    input  logic [`REG_AW-1:0]          ex_pipe_rd_addr,
    input  logic                        ex_pipe_mem_read,
    input  logic                        ex_pipe_mem_write,
    input  logic [`MEM_OP_WIDTH-1:0]    ex_pipe_mem_opcode,
    input  logic                        ex_pipe_unsign,
    input  logic [`XLEN-1:0]            ex_pipe_immediate,
    // EX <--> MEM Pipeline
    input  logic                        mem_pipe_ready,
    input  logic                        mem_pipe_flush,
    output logic                        mem_pipe_valid,
    output logic [`XLEN-1:0]            mem_pipe_pc,
    output logic [`XLEN-1:0]            mem_pipe_instruction,
    output logic [`MEM_OP_WIDTH-1:0]    mem_pipe_mem_opcode,
    output logic                        mem_pipe_mem_read,
    output logic                        mem_pipe_unsign,
    output logic                        mem_pipe_rd_write,
    output logic [`REG_AW-1:0]          mem_pipe_rd_addr,
    output logic [`XLEN-1:0]            mem_pipe_alu_result,
    // EX to other stage
    output logic                        ex_branch,       // jump and taken branch
    output logic [`XLEN-1:0]            ex_branch_pc,    // target pc
    output logic                        ex_rd_write,
    output logic [`REG_AW-1:0]          ex_rd_addr,
    output logic [`XLEN-1:0]            ex_rd_wdata,
    // Data RAM Access
    output logic                        dram_req,
    output logic                        dram_write,
    output logic [`XLEN/8-1:0]          dram_wstrb,
    output logic [`XLEN-1:0]            dram_addr,
    output logic [`XLEN-1:0]            dram_wdata,
    input  logic                        dram_addr_ok
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic ex_pipe_done;
    logic ex_pipe_req;
    logic ex_valid;

    // From/to ALU
    logic [`XLEN-1:0] alu_result;
    logic [`XLEN-1:0] alu_adder_result;
    logic [`XLEN-1:0] alu_src1;
    logic [`XLEN-1:0] alu_src2;

    // Memory Control
    logic       drem_done;
    logic [3:0] wstrb_byte;
    logic [3:0] wstrb_half;
    logic [3:0] wstrb_word;

    // Branch Control
    logic               branch_result_eq;
    logic               branch_eq_success;
    logic               branch_lt_success;
    logic               branch_success;
    logic [`ALU_OP_WIDTH-1:0] branch_cal_opcode;
    logic [`XLEN-1:0]   branch_cal_result;
    logic               exc_ins_addr_mis;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign ex_valid = ex_pipe_valid & ~mem_pipe_flush;
    assign ex_pipe_done = drem_done;
    assign ex_pipe_ready = mem_pipe_ready & ex_pipe_done;
    assign ex_pipe_req = ex_pipe_done & ex_valid;
    assign ex_pipe_flush = ex_branch | mem_pipe_flush;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (rst_b)          mem_pipe_valid <= 1'b0;
        else if (mem_pipe_ready) mem_pipe_valid <= ex_pipe_req;
    end

    always @(posedge clk) begin
        mem_pipe_pc <= ex_pipe_pc;
        mem_pipe_instruction <= ex_pipe_instruction;
        mem_pipe_mem_opcode <= ex_pipe_mem_opcode;
        mem_pipe_mem_read <= ex_pipe_mem_read;
        mem_pipe_unsign <= ex_pipe_unsign;
        mem_pipe_rd_write <= ex_pipe_rd_write;
        mem_pipe_rd_addr <= ex_pipe_rd_addr;
        mem_pipe_alu_result <= alu_result;
    end

    // --------------------------------------
    // Jump/Branch Control Logic
    // --------------------------------------

    // Check if branch is taken or not
    // Reuse the ALU to calculate the branch result
    always @(*) begin
        branch_cal_opcode = '0; // set all the other opcode bits to 0
        branch_cal_opcode[`ALU_OP_SUB] = ex_pipe_branch_opcode[`BRANCH_OP_EQ];
        branch_cal_opcode[`ALU_OP_SLT] = ex_pipe_branch_opcode[`BRANCH_OP_LT] & ~ex_pipe_unsign;
        branch_cal_opcode[`ALU_OP_SLTU] = ex_pipe_branch_opcode[`BRANCH_OP_LT] & ex_pipe_unsign;
    end

    alu branch_cal (
        .alu_opcode(branch_cal_opcode),
        .alu_src1(ex_pipe_rs1_rdata),
        .alu_src2(ex_pipe_rs2_rdata),
        .alu_result(branch_cal_result),
        /* verilator lint_off PINCONNECTEMPTY */
        .alu_adder_result() // Not used
        /* verilator lint_on PINCONNECTEMPTY */
    );

    assign branch_result_eq = ~(|branch_cal_result); // branch_cal_result == 0
    assign branch_eq_success = ex_pipe_branch_opcode[`BRANCH_OP_EQ] &
                               (ex_pipe_branch_opcode[`BRANCH_OP_NEGATE] ^ branch_result_eq);
    assign branch_lt_success = ex_pipe_branch_opcode[`BRANCH_OP_LT] &
                               (ex_pipe_branch_opcode[`BRANCH_OP_NEGATE] ^ branch_cal_result[0]);
    assign branch_success = ex_pipe_branch & (branch_eq_success | branch_lt_success);

    // Calculate the branch/jump address
    // lsb is set to zero for JALR instructions, for other jump/branch instruction
    // lsb should already be zero so there is no harm to set it to zero
    assign ex_branch_pc = {alu_adder_result[`XLEN-1:1], 1'b0};

    // Final branch control signal
    // Note: we still branch/jump even if we have an instruction-address-misaligned exception
    // let the exception handling logic to deal with flushing pipeline
    assign ex_branch = (branch_success | ex_pipe_jump) & ex_valid;

    // generate an instruction-address-misaligned exception
    // if the target address is not aligned to a four-byte boundary
    //assign exc_ins_addr_mis = (branch_success | ex_jump) & |branch_pc[1:0];

    // --------------------------------------
    // Data Ram Access Control
    // --------------------------------------
    assign dram_req = ex_valid & (ex_pipe_mem_read | ex_pipe_mem_write);
    assign dram_write = ex_pipe_mem_write;
    assign dram_addr = alu_adder_result;

    assign dram_wdata = ({`XLEN{ex_pipe_mem_opcode[`MEM_OP_BYTE]}} & {4{ex_pipe_rs2_rdata[7:0]}})  |
                        ({`XLEN{ex_pipe_mem_opcode[`MEM_OP_HALF]}} & {2{ex_pipe_rs2_rdata[15:0]}}) |
                        ({`XLEN{ex_pipe_mem_opcode[`MEM_OP_WORD]}} & ex_pipe_rs2_rdata);

    assign wstrb_byte = 1 << dram_addr[1:0];
    assign wstrb_half = {~dram_addr[1], ~dram_addr[1], dram_addr[1], dram_addr[1]};
    assign wstrb_word = {4{ex_pipe_mem_opcode[`MEM_OP_WORD]}};
    assign dram_wstrb = wstrb_byte | wstrb_half | wstrb_word;

    assign drem_done = dram_req & dram_addr_ok;

    // --------------------------------------
    // ALU src select
    // --------------------------------------
    assign alu_src1 = ex_pipe_alu_src1_sel_pc ? ex_pipe_pc : ex_pipe_rs1_rdata;
    assign alu_src2 = ex_pipe_alu_src2_sel_imm ? ex_pipe_immediate : ex_pipe_rs2_rdata;

    // --------------------------------------
    // Forward logic to ID stage
    // --------------------------------------
    // no need to check if ex_valid, if ex is not valid, then the dependency by nature would be failed.
    assign ex_rd_write = ex_pipe_rd_write;
    assign ex_rd_addr = ex_pipe_rd_addr;
    assign ex_rd_wdata = alu_result;

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    alu u_alu (
        .alu_opcode(ex_pipe_alu_opcode),
        .*);

endmodule
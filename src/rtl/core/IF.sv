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
    logic [`XLEN-1:0] branch_pc_minus4;

    // glue due to pipeline stall/flush
    logic [`XLEN-1:0] backup_instruction;       // a register to hold the instruction in IF stage in case of id is not ready
    logic load_backup_instructions;
    logic backup_instruction_valid;
    logic flush_next_data;

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
    assign if_done = (iram_data_ok & ~flush_next_data) | backup_instruction_valid;
    assign if_req = if_done & if_valid;
    assign if_pipe_ready = ~if_valid | if_req & id_pipe_ready;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        id_pipe_valid <= 1'b0;
        else if (id_pipe_ready) id_pipe_valid <= if_req & ~id_pipe_flush;
    end

    always @(posedge clk) begin
        if (id_pipe_ready & if_req) begin
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
    assign iram_req = if_pipe_ready & rst_b;     // we only read the memory when IF stage is ready
    assign iram_write = 1'b0;
    assign iram_wstrb = '0;
    assign iram_addr = next_pc;
    assign iram_wdata = '0;

    // -------------------------------------------
    // PC logic
    // -------------------------------------------

    assign pc_val = pc;
    assign next_pc = ex_branch ? ex_branch_pc : pc + `XLEN'h4;
    assign branch_pc_minus4 = ex_branch_pc - `XLEN'h4;

    always @(posedge clk) begin
        if (!rst_b) begin
            // because we use next_pc for ram address, we need to minus 4 here
            pc <= `PC_RESET_ADDR - 4;
        end
        else begin
            // update pc to nextPC value when the memory read request is taken and can be moved to next stage
            if (preif_done && if_pipe_ready) pc <= next_pc;

            // If the addr_ok is not valid at the same cycle when the branch request comes,
            // then the request to fetch from the new address will not need to wait and the
            // targetPC (new address to the memory) will be lost because branch instruction is only valid for 1 cycle.
            // One solution is to update the PC register to TargetPC - 4 instead of TargetPC so the nextPC will
            // still be target PC in this case.
            // This is acceptable because the IF stage will not be valid until the read request is accepted.
            // Once the request is accepted then PC becomes TargetPC and IF stage becomes valid in the next cycle.
            else if (ex_branch && !preif_done) pc <= branch_pc_minus4;
        end
    end

    // -------------------------------------------
    // Instruction logic
    // -------------------------------------------

    // Instruction Backup
    // If IF is request to send data to ID but ID is not ready, put the data in backup_instruction and set the valid bit
    // If ID is ready while backup_instruction is valid, clear the valid bit and select the data from backup_instruction

    assign load_backup_instructions = if_req & ~id_pipe_ready & ~backup_instruction_valid;

    always @(posedge clk) begin
        if (!rst_b) backup_instruction_valid <= 1'b0;
        else begin
            if (load_backup_instructions) backup_instruction_valid <= 1'b1;
            else if (id_pipe_ready) backup_instruction_valid <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (load_backup_instructions) backup_instruction <= iram_rdata;
    end

    assign instruction = backup_instruction_valid ? backup_instruction : iram_rdata;

    // Instruction Flushing
    // If ID is flushed whil we are waiting for data, store this information in a register so when the data is available
    // we can flush it

    always @(posedge clk) begin
        if (!rst_b) flush_next_data <= 1'b0;
        else begin
            if (preif_pipe_valid && !iram_data_ok && id_pipe_flush) flush_next_data <= 1'b1;
            else if (flush_next_data & iram_data_ok) flush_next_data <= 1'b0;
        end
    end


    // -------------------------------------------
    // Assertion
    // -------------------------------------------


    // -------------------------------------------
    // Coverage
    // -------------------------------------------

    `ifdef COVERAGE

    // Cover the case that IF is requesting to send data to ID but ID is not ready
    // This will involve using the backup isntruction register
    property if_req_id_not_ready;
        @(posedge clk) disable iff(!rst_b)
        $rose(if_req && ~id_pipe_flush) |-> !id_pipe_ready;
    endproperty
    cov_if_req_id_not_ready: cover property(if_req_id_not_ready);

    // Cover the case that the instruction read is accepted for the new branch target
    property branch_addr_ok;
        @(posedge clk) disable iff(!rst_b)
        $rose(ex_branch) |-> iram_addr_ok;
    endproperty
    cov_branch_addr_ok: cover property(branch_addr_ok);

    // Cover the case that the instruction read is waiting for the new branch target
    property branch_wait_addr_ok;
        @(posedge clk) disable iff(!rst_b)
        $rose(ex_branch) |-> !iram_addr_ok;
    endproperty
    cov_branch_wait_addr_ok: cover property(branch_wait_addr_ok);

    // Cover the case that IF is waiting for a pending read data while a branch request comes
    property branch_while_waiting_data;
        @(posedge clk) disable iff(!rst_b)
        $rose(ex_branch) |-> (preif_pipe_valid && !iram_data_ok);
    endproperty
    cov_branch_while_waiting_data: cover property(branch_while_waiting_data);

    `endif

endmodule
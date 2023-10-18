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

module MEM #(
    parameter SUPPORT_RV32M = 1,
    parameter SUPPORT_ZICSR = 1,
    parameter SUPPORT_TRAP = 1
) (
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
    input  logic [1:0]                  mem_pipe_mem_byte_addr,
    input  logic                        mem_pipe_unsign,
    input  logic                        mem_pipe_rd_write,
    input  logic [`REG_AW-1:0]          mem_pipe_rd_addr,
    input  logic [`XLEN-1:0]            mem_pipe_alu_result,
    input  logic                        mem_pipe_csr_write,
    input  logic                        mem_pipe_csr_set,
    input  logic                        mem_pipe_csr_clear,
    input  logic                        mem_pipe_csr_read,
    input  logic [`XLEN-1:0]            mem_pipe_csr_info,
    input  logic [11:0]                 mem_pipe_csr_addr,
    input  logic                        mem_pipe_mret,
    input  logic                        mem_pipe_mul,
    input  logic                        mem_pipe_exc_pending,
    input  logic [3:0]                  mem_pipe_exc_code,
    input  logic [`XLEN-1:0]            mem_pipe_exc_tval,
    input  logic                        mem_pipe_exc_interrupt,
    // MEM <--> WB Pipeline
    input  logic                        wb_pipe_ready,
    input  logic                        wb_pipe_flush,
    output logic                        wb_pipe_valid,
    output logic [`XLEN-1:0]            wb_pipe_pc,
    output logic [`XLEN-1:0]            wb_pipe_instruction,
    output logic                        wb_pipe_rd_write,
    output logic [`REG_AW-1:0]          wb_pipe_rd_addr,
    output logic [`XLEN-1:0]            wb_pipe_rd_data,
    output logic                        wb_pipe_csr_write,
    output logic                        wb_pipe_csr_set,
    output logic                        wb_pipe_csr_clear,
    output logic                        wb_pipe_csr_read,
    output logic [`XLEN-1:0]            wb_pipe_csr_info,
    output logic [11:0]                 wb_pipe_csr_addr,
    output logic                        wb_pipe_mret,
    output logic                        wb_pipe_mul,
    output logic                        wb_pipe_exc_pending,
    output logic [3:0]                  wb_pipe_exc_code,
    output logic [`XLEN-1:0]            wb_pipe_exc_tval,
    output logic                        wb_pipe_exc_interrupt,
    // MEM to other stage
    output logic                        mem_rd_write,
    output logic [`REG_AW-1:0]          mem_rd_addr,
    output logic [`XLEN-1:0]            mem_rd_wdata,
    output logic                        mem_mem_read_wait,
    output logic                        mem_csr_read,
    output logic                        mem_mul,
    // Data RAM Access
    input  logic                        dram_rvalid,
    input  logic [`XLEN-1:0]            dram_rdata
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic mem_done;
    logic mem_req;
    logic mem_valid;

    // Memory Read Control
    logic               load_done;
    logic               load_wait;
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
    // Pipeline Control
    // --------------------------------------

    assign mem_valid = mem_pipe_valid & ~wb_pipe_flush;
    assign mem_done = ~load_wait;
    assign mem_req = mem_done & mem_valid;

    assign mem_pipe_ready = ~mem_valid | mem_req & wb_pipe_ready;
    assign mem_pipe_flush = (mem_pipe_valid & (mem_pipe_exc_pending | mem_pipe_mret)) | wb_pipe_flush;

    // Pipeline Register Update
    always @(posedge clk) begin
        if      (!rst_b)        wb_pipe_valid <= 1'b0;
        else if (wb_pipe_ready) wb_pipe_valid <= mem_req;
    end

    always @(posedge clk) begin
        if (wb_pipe_ready & mem_req) begin
            wb_pipe_pc <= mem_pipe_pc;
            wb_pipe_instruction <= mem_pipe_instruction;
            wb_pipe_rd_write <= mem_pipe_rd_write;
            wb_pipe_rd_addr <= mem_pipe_rd_addr;
            wb_pipe_rd_data <= rd_data;
        end
    end

    // CSR
    generate
    if (SUPPORT_ZICSR) begin: gen_csr_pipe
        always @(posedge clk) begin
            if (wb_pipe_ready && mem_req) begin
                wb_pipe_csr_write <= mem_pipe_csr_write;
                wb_pipe_csr_set   <= mem_pipe_csr_set;
                wb_pipe_csr_clear <= mem_pipe_csr_clear;
                wb_pipe_csr_read  <= mem_pipe_csr_read;
                wb_pipe_csr_info  <= mem_pipe_csr_info;
                wb_pipe_csr_addr  <= mem_pipe_csr_addr;
            end
        end
    end
    else begin: no_csr_pipe
        assign wb_pipe_csr_write = 1'b0;
        assign wb_pipe_csr_set = 1'b0;
        assign wb_pipe_csr_clear = 1'b0;
        assign wb_pipe_csr_read  = 1'b0;
        assign wb_pipe_csr_info = `XLEN'b0;
        assign wb_pipe_csr_addr  = 12'b0;
    end
    endgenerate

    // Exception/Interrupt
    generate
    if (SUPPORT_TRAP) begin: gen_trap_pipe
        always @(posedge clk) begin
            if (wb_pipe_ready && mem_req) begin
                wb_pipe_mret <= mem_pipe_mret;
                wb_pipe_exc_pending <= mem_pipe_exc_pending;
                wb_pipe_exc_code <= mem_pipe_exc_code;
                wb_pipe_exc_interrupt <= mem_pipe_exc_interrupt;
            end
        end
    end
    else begin: no_trap_pipe
        assign wb_pipe_mret = 1'b0;
        assign wb_pipe_exc_pending = 1'b0;
        assign wb_pipe_exc_code = 4'b0;
        assign wb_pipe_exc_interrupt = 1'b0;
    end
    endgenerate

    // RV32M Extension
    generate
    if (SUPPORT_RV32M) begin: gen_rv32m_pipe
        always @(posedge clk) begin
            if (wb_pipe_ready && mem_req) begin
                wb_pipe_mul <= mem_pipe_mul;
            end
        end
    end
    else begin: no_rv32m_pipe
        assign wb_pipe_mul = 1'b0;
    end
    endgenerate

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

    assign lb_data = ({8{mem_pipe_mem_byte_addr[1:0] == 0}} & dram_rdata[ 7: 0]) |
                     ({8{mem_pipe_mem_byte_addr[1:0] == 1}} & dram_rdata[15: 8]) |
                     ({8{mem_pipe_mem_byte_addr[1:0] == 2}} & dram_rdata[23:16]) |
                     ({8{mem_pipe_mem_byte_addr[1:0] == 3}} & dram_rdata[31:24]);

    assign lh_data = mem_pipe_mem_byte_addr[1] ? dram_rdata[31:16] : dram_rdata[15:0];

    assign load_data = ({`XLEN{is_lb}}  & lb_ext_data)  |
                       ({`XLEN{is_lbu}} & lbu_ext_data) |
                       ({`XLEN{is_lh}}  & lh_ext_data)  |
                       ({`XLEN{is_lhu}} & lhu_ext_data) |
                       ({`XLEN{is_lw}}  & dram_rdata) ;

    assign load_done = mem_valid & mem_pipe_mem_read & dram_rvalid;
    assign load_wait = mem_pipe_mem_read & ~load_done;

    // --------------------------------------
    // Registr write back data selection
    // --------------------------------------

    assign rd_data = mem_pipe_mem_read ? load_data : mem_pipe_alu_result;

    // --------------------------------------
    // Forward logic to ID stage
    // --------------------------------------

    assign mem_rd_write = mem_pipe_rd_write & mem_pipe_valid;
    assign mem_rd_addr  = mem_pipe_rd_addr;
    assign mem_rd_wdata = rd_data;
    assign mem_csr_read = mem_pipe_csr_read & mem_pipe_valid;
    assign mem_mul      = mem_pipe_mul & mem_pipe_valid;
    assign mem_mem_read_wait = mem_pipe_mem_read & ~dram_rvalid;


endmodule
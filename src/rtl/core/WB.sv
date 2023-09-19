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

module WB #(
    parameter SUPPORT_ZICSR = 1,
    parameter SUPPORT_TRAP = 1
) (
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
    input  logic                        wb_pipe_csr_write,
    input  logic                        wb_pipe_csr_set,
    input  logic                        wb_pipe_csr_clear,
    input  logic                        wb_pipe_csr_read,
    input  logic [`XLEN-1:0]            wb_pipe_csr_info,
    input  logic [11:0]                 wb_pipe_csr_addr,
    input  logic                        wb_pipe_mret,
    input  logic                        wb_pipe_exc_pending,
    input  logic [3:0]                  wb_pipe_exc_code,
    input  logic [`XLEN-1:0]            wb_pipe_exc_tval,
    input  logic                        wb_pipe_exc_interrupt,
    // To register writeback
    output logic                        wb_rd_write,
    output logic [`REG_AW-1:0]          wb_rd_addr,
    output logic [`XLEN-1:0]            wb_rd_wdata,
    // To IF stage
    output logic                        wb_trap,
    output logic [`XLEN-1:0]            wb_trap_pc,
    // Interrupt input
    input  logic                        external_interrupt,
    input  logic                        software_interrupt,
    input  logic                        timer_interrupt
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Pipeline Control
    logic wb_done;
    logic wb_req;
    logic wb_valid;

    // CSR
    logic [`XLEN-1:0] csr_read_data;
    logic             csr_rd_mstatus_mie;
    logic             csr_rd_mstatus_mpie;
    logic             csr_rd_mie_msie;
    logic             csr_rd_mie_mtie;
    logic             csr_rd_mie_meie;
    logic [`XLEN-3:0] csr_rd_mtvec_base;
    logic [1:0]       csr_rd_mtvec_mode;
    logic [`XLEN-1:0] csr_rd_mepc_mepc;

    // Trap
    logic             csr_wr_mstatus_mie;
    logic             csr_wr_mstatus_mpie;
    logic [`XLEN-1:0] csr_wr_mepc_mepc;
    logic [`XLEN-1:0] csr_wr_mtval_mtval;
    logic [`XLEN-2:0] csr_wr_mcause_exception_code;
    logic             csr_wr_mcause_interrupt;
    logic             csr_set_mip_msip;
    logic             csr_set_mip_mtip;
    logic             csr_set_mip_meip;
    logic             ent_trap;
    logic             ext_trap;

    // --------------------------------------
    // Pipeline Logic
    // --------------------------------------

    // Pipeline Control
    assign wb_valid = wb_pipe_valid;
    assign wb_done = 1'b1;
    assign wb_pipe_ready = wb_done;
    assign wb_req = wb_done & wb_valid;
    assign wb_pipe_flush = wb_pipe_valid & (wb_pipe_exc_pending | wb_pipe_mret);

    // --------------------------------------
    // register write logic
    // --------------------------------------
    assign wb_rd_write = wb_pipe_rd_write & wb_pipe_valid;
    assign wb_rd_addr  = wb_pipe_rd_addr;
    assign wb_rd_wdata = wb_pipe_csr_read ? csr_read_data : wb_pipe_rd_data;

    // --------------------------------------
    // Module instantiation
    // --------------------------------------

    generate
    if (SUPPORT_ZICSR) begin: gen_csr
        csr u_csr (
        .clk            (clk),
        .rst_b          (rst_b),
        .csr_write      (wb_pipe_csr_write),
        .csr_set        (wb_pipe_csr_set),
        .csr_clear      (wb_pipe_csr_clear),
        .csr_read       (wb_pipe_csr_read),
        .csr_info       (wb_pipe_csr_info),
        .csr_addr       (wb_pipe_csr_addr),
        .csr_read_data  (csr_read_data),
        .*
        );
    end
    else begin: no_csr
        assign csr_read_data = `XLEN'b0;
        assign csr_rd_mstatus_mie = 1'b0;
        assign csr_rd_mstatus_mpie = 1'b0;
        assign csr_rd_mie_msie = 1'b0;
        assign csr_rd_mie_mtie = 1'b0;
        assign csr_rd_mie_meie = 1'b0;
        assign csr_rd_mtvec_base = {`XLEN-3{1'b0}};
        assign csr_rd_mtvec_mode = 2'b0;
        assign csr_rd_mepc_mepc = 1'b0;
    end
    endgenerate


    generate
    if (SUPPORT_TRAP) begin: gen_trap
        trap u_trap (
            .pc         (wb_pipe_pc),
            .instruction(wb_pipe_instruction),
            .tval       (wb_pipe_exc_tval),
            .valid      (wb_pipe_valid),
            .mret       (wb_pipe_mret),
            .exc_pending(wb_pipe_exc_pending),
            .exc_code   (wb_pipe_exc_code),
            .exc_tval   (wb_pipe_exc_tval),
            .exc_interrupt(wb_pipe_exc_interrupt),
            .trap       (wb_trap),
            .trap_pc    (wb_trap_pc),
            .*
        );
    end
    endgenerate

endmodule
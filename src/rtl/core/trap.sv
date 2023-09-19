/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 09/15/2023
 *
 * ------------------------------------------------------------------------------------------------
 * trap: trap controller. Handling all the exceptions and interrupts
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"
`include "riscv_isa.svh"

module trap (
    input  logic                clk,
    input  logic                rst_b,

    // information
    input  logic [`XLEN-1:0]    pc,
    input  logic [`XLEN-1:0]    instruction,
    input  logic [`XLEN-1:0]    tval,

    // Interrupt
    input  logic                software_interrupt,
    input  logic                timer_interrupt,
    input  logic                external_interrupt,

    // trap
    input  logic                valid,
    input  logic                mret,
    input  logic                exc_pending,
    input  logic [3:0]          exc_code,
    input  logic [`XLEN-1:0]    exc_tval,
    input  logic                exc_interrupt,

    // output control
    output logic                trap,       // trap request to change pc
    output logic [`XLEN-1:0]    trap_pc,    // trap pc target
    output logic                ent_trap,   // enter trap
    output logic                ext_trap,   // exit trap

    // CSR write data from trap controller to CSR
    output logic                csr_wr_mstatus_mie,
    output logic                csr_wr_mstatus_mpie,
    output logic [`XLEN-1:0]    csr_wr_mepc_mepc,
    output logic [`XLEN-1:0]    csr_wr_mtval_mtval,
    output logic [`XLEN-2:0]    csr_wr_mcause_exception_code,
    output logic                csr_wr_mcause_interrupt,
    output logic                csr_set_mip_msip,
    output logic                csr_set_mip_mtip,
    output logic                csr_set_mip_meip,
    // CSR read data from CSR to trap controller
    input logic                 csr_rd_mstatus_mie,
    input logic                 csr_rd_mstatus_mpie,
    input logic                 csr_rd_mie_msie,
    input logic                 csr_rd_mie_mtie,
    input logic                 csr_rd_mie_meie,
    input logic [`XLEN-3:0]     csr_rd_mtvec_base,
    input logic [1:0]           csr_rd_mtvec_mode,
    input logic [`XLEN-1:0]     csr_rd_mepc_mepc
);

    logic [`XLEN-1:0] mtvec_vectored;
    logic [`XLEN-1:0] mtvec_nonvectored;
    logic [`XLEN-1:0] ent_pc; // new pc for entering exception/interrupt
    logic [`XLEN-1:0] ret_pc; // new pc for returning exception/interrupt

    logic [3:0]       interrupt_code;
    logic             global_interrupt_pending;
    logic             final_software_interrupt;
    logic             final_timer_interrupt;
    logic             final_external_interrupt;
    logic             final_interrupt;

    // -----------------------------------
    // entering trap
    // -----------------------------------

    // Start to execute from PC address defined in mtvec
    assign mtvec_vectored = {csr_rd_mtvec_base, 2'b0} + {exc_code, 2'b0};
    assign mtvec_nonvectored = {csr_rd_mtvec_base, 2'b0};
    // New PC value:
    // If mode = 0, both exception and interrupt use mtvec_base
    // If mode = 1, exception use mtvec_base while interrupt use mtvec_base + 4*cause
    // => This is the same as if mode == 1 && interrupt, use mtvec_base + 4*cause else use mtvec_base
    assign ent_pc = (exc_interrupt && (csr_rd_mtvec_mode == 2'b1)) ? mtvec_nonvectored : mtvec_nonvectored;
    // Update mcause register
    assign csr_wr_mcause_interrupt = exc_interrupt;
    assign csr_wr_mcause_exception_code[3:0] = exc_interrupt ? interrupt_code : exc_code;
    assign csr_wr_mcause_exception_code[`XLEN-2:4] = 0; // Not used in our design

    assign interrupt_code = {4{software_interrupt}} & `EC_M_SOFTWARE_INTERRUPT |
                            {4{timer_interrupt}}    & `EC_M_TIMER_INTERRUPT    |
                            {4{external_interrupt}} & `EC_M_EXTERNAL_INTERRUPT ;


    // update mepc register
    assign csr_wr_mepc_mepc = pc;

    // update mtval register
    assign csr_wr_mtval_mtval = exc_tval;

    // update mstatus
    // Coded in later session.

    // update mip register
    assign csr_set_mip_msip = software_interrupt;
    assign csr_set_mip_mtip = timer_interrupt;
    assign csr_set_mip_meip = external_interrupt;

    // deternime if interrupt is masked or not
    assign global_interrupt_pending  = valid & exc_interrupt & csr_rd_mstatus_mie;
    assign final_software_interrupt = global_interrupt_pending & software_interrupt & csr_rd_mie_msie;
    assign final_timer_interrupt = global_interrupt_pending & timer_interrupt & csr_rd_mie_mtie;
    assign final_external_interrupt = global_interrupt_pending & external_interrupt & csr_rd_mie_meie;
    assign final_interrupt = final_software_interrupt | final_timer_interrupt | final_external_interrupt;

    // -----------------------------------
    // exiting trap
    // -----------------------------------
    assign ret_pc = csr_rd_mepc_mepc;

    // -----------------------------------
    // Final control signal
    // -----------------------------------

    // determine if trap to PC is taken or not
    assign ent_trap = valid & (exc_pending | final_interrupt);
    assign ext_trap = valid & mret;
    assign trap = ent_trap | ext_trap;

    //  determine trap_pc
    assign trap_pc = ext_trap ? ret_pc : ent_pc;

    // update mstatus
    assign csr_wr_mstatus_mpie = ext_trap ? 1'b1 : csr_rd_mstatus_mie;
    assign csr_wr_mstatus_mie = ext_trap ? csr_wr_mstatus_mpie : 1'b0;

endmodule

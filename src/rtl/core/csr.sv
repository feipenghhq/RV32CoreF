/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 09/12/2023
 *
 * ------------------------------------------------------------------------------------------------
 * CSR: Control and Status Register Module
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"

module csr (
    input  logic                clk,
    input  logic                rst_b,
    // CSR read/write bus
    input  logic                csr_write,
    input  logic                csr_set,
    input  logic                csr_clear,
    input  logic                csr_read,
    input  logic [`XLEN-1:0]    csr_info,
    input  logic [11:0]         csr_addr,
    output logic [`XLEN-1:0]    csr_read_data
);

    logic [`XLEN-1:0] csr_write_data;
    logic [`XLEN-1:0] csr_set_data;
    logic [`XLEN-1:0] csr_clear_data;

    logic [`XLEN-1:0] csr_wdata;

    assign csr_write_data = csr_info; // this will be rs1 value
    assign csr_set_data   = csr_info | csr_read_data;
    assign csr_clear_data = ~csr_info & csr_read_data;

    assign csr_wdata = ({`XLEN{csr_write}} & csr_write_data) |
                       ({`XLEN{csr_set}}   & csr_set_data)   |
                       ({`XLEN{csr_clear}} & csr_clear_data) ;

endmodule
/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/13/2023
 *
 * ------------------------------------------------------------------------------------------------
 * regfile: Register File
 * ------------------------------------------------------------------------------------------------
 */

`include "config.svh"

module regfile #(
    parameter R0_ZERO = 0
)(
    input  logic                clk,
    // RS1 read port
    input  logic [`REG_AW-1:0]  rs1_addr,
    output logic [`XLEN-1:0]    rs1_rdata,
    // RS2 read port
    input  logic [`REG_AW-1:0]  rs2_addr,
    output logic [`XLEN-1:0]    rs2_rdata,
    // RD write port
    input  logic [`REG_AW-1:0]  rd_addr,
    input  logic [`XLEN-1:0]    rd_wdata,
    input  logic                rd_write
);

    reg [`XLEN-1:0] register[`REG_NUM];


    generate
        if (R0_ZERO) begin: gen_r0_zero
            assign rs1_rdata = (rs1_addr == 0) ? `XLEN'b0 : register[rs1_addr];
            assign rs2_rdata = (rs2_addr == 0) ? `XLEN'b0 : register[rs2_addr];
        end
        else begin: gen_no_r0_zero
            assign rs1_rdata = register[rs1_addr];
            assign rs2_rdata = register[rs2_addr];
        end
    endgenerate

    // RD
    always @(posedge clk) begin
        if (rd_write) begin
            register[rd_addr] <= rd_wdata;
        end
    end

endmodule

/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 10/15/2023
 *
 * ------------------------------------------------------------------------------------------------
 * Multiplier for CPU
 * ------------------------------------------------------------------------------------------------
 * Currently our CPU design is targeting FPGA so we use * operator directly for multiplication.
 * The mulpiler has 2 stage pipeline with input and output register so the logic can be mapped
 * to DSP logic in FPGA
 *
 * In order to save resources, we use a single multiplier for both signed and unsigned operations.
 * To do this, we use a 33 bit multiplier instead of a 32 bit multiplier.
 * For signed operations, we signed extend the 32 bit to 33 bit so the data will be treated as signed.
 * For unsigned operations, we unsign extend the 32 bit to 33 bit so the data will be treated as unsigned.
 *
 * mul_opcode used in mul_opcode signal:
 *     MUL       2'b00
 *     MULH      2'b01
 *     MULHSU    2'b10
 *     MULHU     2'b11
 * ------------------------------------------------------------------------------------------------
 */

`include "core.svh"
`include "config.svh"

module multiplier (
    input                       clk,
    input                       rst_b,
    input                       mul,
    input  [`MUL_OP_WIDTH-1:0]  mul_opcode,
    input  [`XLEN-1:0]          mul_src1,
    input  [`XLEN-1:0]          mul_src2,
    output [`XLEN-1:0]          mul_result
);

    // ---------------------------------
    // Signal Declaration
    // ---------------------------------

    // stage 0 (EX/MEM pipeline)
    logic [`XLEN:0] src1_sign_ext;
    logic [`XLEN:0] src2_sign_ext;
    logic [`XLEN:0] src1_unsign_ext;
    logic [`XLEN:0] src2_unsign_ext;

    logic signed [`XLEN:0] src1_s0;
    logic signed [`XLEN:0] src2_s0;
    logic [`MUL_OP_WIDTH-1:0] mul_opcode_s0;

    // stage 1 (MEM/WB pipeline)
    logic signed [(`XLEN+1)*2-1:0]  result_s1;
    logic [`MUL_OP_WIDTH-1:0] mul_opcode_s1;

    // ---------------------------------
    // Main logic
    // ---------------------------------

    // -------------------------------
    // Pipeline stage 0
    // -------------------------------

    // sign and unsigned extension
    assign src1_sign_ext   = {mul_src1[`XLEN-1], mul_src1};
    assign src2_sign_ext   = {mul_src2[`XLEN-1], mul_src2};
    assign src1_unsign_ext = {1'b0, mul_src1};
    assign src2_unsign_ext = {1'b0, mul_src2};

    // mul_src1 extension:
    //   - MULHU: mul_opcode[1:0] == 2'b11
    always @(posedge clk) src1_s0 <= (mul_opcode[1:0] == 2'b11) ? src1_unsign_ext : src1_sign_ext;

    // mul_src1 extension:
    //  - MULHSU => mul_opcode[1:0] == 2'b10
    //  - MULHU  => mul_opcode[1:0] == 2'b11
    //  => mul_opcode[1] == 1
    always @(posedge clk) src2_s0 <= mul_opcode[1] ? src2_unsign_ext : src2_sign_ext;

    // opcode
    always @(posedge clk) mul_opcode_s0 <= mul_opcode;

    // -------------------------------
    // Pipeline stage 1
    // -------------------------------

    // calculate the result
    always @(posedge clk) result_s1 <= src1_s0 * src2_s0;
    always @(posedge clk) mul_opcode_s1 <= mul_opcode_s0;

    // process the output data (pipeline register is EX/MEM stage)
    assign mul_result = (mul_opcode_s1 == 2'b00) ? result_s1[`XLEN-1:0] : result_s1[2*`XLEN-1:`XLEN];

endmodule
/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 10/15/2023
 *
 * ------------------------------------------------------------------------------------------------
 * Divider for CPU
 * ------------------------------------------------------------------------------------------------
 * The simpliest algorithm for divide operation for hardware is to mimic the Long division.
 * This divider design used the long division algorithm.
 *
 * To save resources, we use a single divider for both signed and unsigned operations.
 * The data will be covered to unsigned values for division and post processed to match with the
 * corresponding signed result
 *
 * mul_div_opcode used in mul_div_opcode signal:
 *     DIV       2'b00
 *     DIVU      2'b01
 *     REM       2'b10
 *     REMU      2'b11
 *
 * Example used in here is 15/2 => 1111 / 00100
 *   - quotient:  7 (0111)
 *   - remainder: 1 (0001)
 * In the example, XLEN = 4
 * ------------------------------------------------------------------------------------------------
 */

/*
 Basic Equations of Division:

   dividend = divisor × quotient + remainder

 From RISC-V Specification:
   DIV and DIVU perform an XLEN bits by XLEN bits signed and unsigned integer division of rs1 by
   rs2, rounding towards zero.
   REM and REMU provide the remainder of the corresponding division
   operation. For REM, the sign of the result equals the sign of the dividend.

 Condiser the example (calculated in python):
               Q  R
    15 /  2 =  7 (1)
    15 / -2 = -7 (1)
   -15 /  2 = -7 (-1)
   -15 / -2 =  7 (-1)

If we covert the signed value to unsigned value for division, then we need to post process the result
Based on the above observation, we need to
   1. inverse the quotient if sign(divident) != sign(divisor)
   2. inverse the remainder if divident < 0
Note, if divide by zero, then don't negate the quotient as quotient = divident

 */

`include "core.svh"
`include "config.svh"

module divider (
    input               clk,
    input               rst_b,
    input               div_req,
    input  [1:0]        div_opcode,
    input  [`XLEN-1:0]  div_src1,
    input  [`XLEN-1:0]  div_src2,
    output [`XLEN-1:0]  div_result,
    output              div_valid,
    output              div_ready
);

    // ---------------------------------
    // Signal Declaration
    // ---------------------------------

    logic                   div_req_taken;
    logic                   div_running;
    logic                   div_complete;

    logic [`XLEN-1:0]       dividend;
    logic [`XLEN-1:0]       divisor;    // dividend / divisor
    logic [`XLEN-1:0]       quotient;
    logic                   is_rem;     // 1 - REM/REMU, 0 - DIV/DIVU

    logic                   divide_by_zero;
    logic                   quotient_negate;
    logic                   remainder_negate;

    logic                   signed_div;
    logic                   dividend_sign_bit;
    logic                   divisor_sign_bit;
    logic [`XLEN-1:0]       dividend_unsign;
    logic [`XLEN-1:0]       divisor_unsign;

    logic [$clog2(`XLEN+1)-1:0] iterations;

    logic [2*`XLEN-1:0]     extended_dividend;
    logic [2*`XLEN-1:0]     extended_divisor;
    logic [2*`XLEN-1:0]     substract_result;
    logic                   substract_positive;

    logic [`XLEN-1:0]       quotient_final;
    logic [`XLEN-1:0]       remainder_final;

    // ---------------------------------
    // Main logic
    // ---------------------------------

    assign div_req_taken = div_req & div_ready;

    // stores the request information into register

    assign divide_by_zero = (div_src2 == 0);

    always @(posedge clk) begin
        if (div_req_taken) begin
            quotient_negate  <= signed_div & (dividend_sign_bit ^ divisor_sign_bit) & ~divide_by_zero;
            remainder_negate <= signed_div & dividend_sign_bit;
            is_rem <= div_opcode[1];
        end
    end

    // we use unsigned divider so we need to convert NEGATIVE number to POSITIVE number
    assign dividend_sign_bit = div_src1[`XLEN-1];
    assign divisor_sign_bit  = div_src2[`XLEN-1];
    assign dividend_unsign = ~div_src1 + 1'b1;
    assign divisor_unsign  = ~div_src2 + 1'b1;
    assign signed_div = ~div_opcode[0];

    // Adjust the width of dividend and dividor to make them aligned for the subtraction.

    // Need to have one additional sign bit and the sign bit should be 0.
    // This is mainly for unsigned division because for unsigned division,
    // the entire 32 bits are taken as unsigned so we need to
    // have one additional bit to indicate that the substraction is unsigned.

    // Example for 15 / 2:
    // extended_dividend = 0_0001111
    // extended_divisor  = 0_0010000
    // Example for unsigned 20 / -6 (-6 actually becomes a very large positive number in unsigned notation)
    // extended_dividend = 0_0010100
    // extended_divisor  = 0_1111010
                             // sign bit
    assign extended_dividend = {1'b0, {(`XLEN-1){1'b0}}, dividend};
    assign extended_divisor  = {1'b0, divisor, {(`XLEN-1){1'b0}}};

    // Substract the dividend and divisor (similar to the long division)
    assign substract_result = extended_dividend - (extended_divisor >> iterations);
    assign substract_positive = ~substract_result[2*`XLEN-1];

    // update values
    always @(posedge clk) begin
        if (!rst_b) begin
            quotient <= 0;
            iterations <= 0;
            dividend <= 0;
            divisor <= 0;
        end
        else if (div_req_taken) begin
            quotient <= 0;
            iterations <= 0;
            dividend <= (signed_div && dividend_sign_bit) ? dividend_unsign : div_src1;
            divisor  <= (signed_div && divisor_sign_bit)  ? divisor_unsign  : div_src2;
        end
        else if (div_running) begin
            // left shift quotient by 1 (because the new bit will be added to the right of the quotient)
            quotient <= quotient << 1;
            // if divident - divisor > 0, add 1 to quotient
            quotient[0] <= substract_positive;
            // update divident if the divisor is subtracted.
            dividend <= substract_positive ? substract_result[`XLEN-1:0] : dividend;
            // update the iteration
            iterations <= iterations + 1'b1;
        end
    end

    // flow control
    assign div_complete = (iterations == `XLEN);

    always @(posedge clk) begin
        if (!rst_b) div_running <= 1'b0;
        else begin
            if      (div_req_taken) div_running <= 1'b1;
            else if (div_complete)  div_running <= 1'b0;
        end
    end

    assign div_ready = ~div_running;
    assign div_valid = div_complete;

    // processing for final value
    // adjust the result based on the sign of the divident/divisor
    assign quotient_final = quotient_negate ? (~quotient + 1'b1) : quotient;
    assign remainder_final = remainder_negate ? (~dividend + 1'b1) : dividend;

    // assign the correct value to output
    assign div_result = is_rem ? remainder_final : quotient_final;

endmodule

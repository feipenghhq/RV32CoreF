/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/30/2023
 *
 * ------------------------------------------------------------------------------------------------
 * gpio: a simple memory mapped gpio
 * ------------------------------------------------------------------------------------------------
 */

// Note: this GPIO is actually not working functionality. It is used to provide an input/output
// for FPGA top level for synthesis.

module gpio #(
    parameter WIDTH = 8,
    parameter AW = 12,
    parameter DW = 32
) (
    input  logic                clk,
    // memory mapped interface
    input  logic                gpio_req,
    input  logic                gpio_write,
    input  logic [DW/8-1:0]     gpio_wstrb, // not used
    input  logic [AW-1:0]       gpio_addr,
    input  logic [DW-1:0]       gpio_wdata,
    output logic                gpio_addr_ok,
    output logic                gpio_data_ok,
    output logic [DW-1:0]       gpio_rdata,
    // GPIO
    inout  [WIDTH-1:0]          GPIO
);

    logic               gpio_wen;
    logic [WIDTH-1:0]   gpio_in;
    logic [WIDTH-1:0]   gpio_out;

    assign GPIO = gpio_wen ? gpio_out : {WIDTH{1'bz}};
    assign gpio_wen = gpio_req & gpio_write;
    assign gpio_in = GPIO;

    assign gpio_addr_ok = 1'b1;
    assign gpio_rdata = gpio_in;

    always @(posedge clk) begin
        gpio_data_ok <= gpio_req & gpio_write;
    end

endmodule
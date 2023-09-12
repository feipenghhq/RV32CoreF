/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/30/2023
 *
 * ------------------------------------------------------------------------------------------------
 * minisoc: A mini soc for testbench and FPGA synthesis
 * ------------------------------------------------------------------------------------------------
 */

// Memory Map of the minisoc
// 0x00000000 - 0x1FFFFFFF      RAM
// 0x20000000 - 0x2FFFFFFF      GPIO

`include "config.svh"

module minisoc #(
    parameter RAM_AW = 12
)(
    input       clk,
    input       rst_b,
    inout [7:0] GPIO
);

    // --------------------------------------
    //  Signal Definition
    // --------------------------------------

    // Instruction RAM Bus
    logic                iram_req;
    logic                iram_write;
    logic [`XLEN/8-1:0]  iram_wstrb;
    logic [`XLEN-1:0]    iram_addr;
    logic [`XLEN-1:0]    iram_wdata;
    logic                iram_ready;
    logic                iram_rvalid;
    logic [`XLEN-1:0]    iram_rdata;

    // Data RAM Bus
    logic                dram_req;
    logic                dram_write;
    logic [`XLEN/8-1:0]  dram_wstrb;
    logic [`XLEN-1:0]    dram_addr;
    logic [`XLEN-1:0]    dram_wdata;
    logic                dram_ready;
    logic                dram_rvalid;
    logic [`XLEN-1:0]    dram_rdata;

    // bus connecting data ram
    logic                data_req;
    logic                data_write;
    logic [`XLEN/8-1:0]  data_wstrb;
    logic [`XLEN-1:0]    data_addr;
    logic [`XLEN-1:0]    data_wdata;
    logic                data_ready;
    logic                data_rvalid;
    logic [`XLEN-1:0]    data_rdata;

    // bus connecting gpio
    logic                gpio_req;
    logic                gpio_write;
    logic [`XLEN/8-1:0]  gpio_wstrb;
    logic [`XLEN-1:0]    gpio_addr;
    logic [`XLEN-1:0]    gpio_wdata;
    logic                gpio_ready;
    logic                gpio_rvalid;
    logic [`XLEN-1:0]    gpio_rdata;

    logic                decode_is_gpio;
    logic                decode_is_dram;

    // --------------------------------------
    // Decode logic for GPIO and data RAM
    // --------------------------------------

    assign decode_is_dram = (dram_addr[31:28] == 4'h1) || (dram_addr[31:28] == 4'h0);
    assign decode_is_gpio = dram_addr[31:28] == 4'h2;

    assign gpio_req   = dram_req & decode_is_gpio;
    assign gpio_write = dram_write;
    assign gpio_wstrb = dram_wstrb;
    assign gpio_addr  = dram_addr;
    assign gpio_wdata = dram_wdata;

    assign data_req   = dram_req & decode_is_dram;
    assign data_write = dram_write;
    assign data_wstrb = dram_wstrb;
    assign data_addr  = dram_addr;
    assign data_wdata = dram_wdata;

    assign dram_ready = (decode_is_gpio & gpio_ready) | (decode_is_dram & data_ready);
    // For simplicity, we just OR the rvalid assuming the following conditions hold true:
    //  1.) rvalid is onehot and only one target receives the request each time.
    //  2.) target device should only drive rvalid to high whenever its data is ready.
    assign dram_rvalid = gpio_rvalid | data_rvalid;
    assign dram_rdata = ({`XLEN{gpio_rvalid}} & gpio_rdata) |
                        ({`XLEN{dram_rvalid}} & data_rdata) ;

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    core u_core (.*);

    parameter RAM_AW_INT = RAM_AW - 2; // address in terms of word address

    // create a 4KB RAM to hold both the instruction and data
    ram  #(.AW(RAM_AW_INT), .DW(`XLEN))
    memory
    (
        .clk(clk),
        .instr_req(iram_req),
        .instr_write(iram_write),
        .instr_wstrb(iram_wstrb),
        .instr_addr(iram_addr[RAM_AW-1:2]),
        .instr_wdata(iram_wdata),
        .instr_ready(iram_ready),
        .instr_rvalid(iram_rvalid),
        .instr_rdata(iram_rdata),
        .data_req(data_req),
        .data_write(data_write),
        .data_wstrb(data_wstrb),
        .data_addr(data_addr[RAM_AW-1:2]),
        .data_wdata(data_wdata),
        .data_ready(data_ready),
        .data_rvalid(data_rvalid),
        .data_rdata(data_rdata)
    );

    gpio #(.AW(16), .DW(`XLEN))
    u_gpio
    (
        .clk(clk),
        .gpio_req(gpio_req),
        .gpio_write(gpio_write),
        .gpio_wstrb(gpio_wstrb),
        .gpio_addr(gpio_addr[15:0]),
        .gpio_wdata(gpio_wdata),
        .gpio_ready(gpio_ready),
        .gpio_rvalid(gpio_rvalid),
        .gpio_rdata(gpio_rdata),
        .GPIO(GPIO)
    );

endmodule
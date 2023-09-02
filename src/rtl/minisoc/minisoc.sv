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

module minisoc (
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
    logic                iram_addr_ok;
    logic                iram_data_ok;
    logic [`XLEN-1:0]    iram_rdata;

    // Data RAM Bus
    logic                dram_req;
    logic                dram_write;
    logic [`XLEN/8-1:0]  dram_wstrb;
    logic [`XLEN-1:0]    dram_addr;
    logic [`XLEN-1:0]    dram_wdata;
    logic                dram_addr_ok;
    logic                dram_data_ok;
    logic [`XLEN-1:0]    dram_rdata;

    // bus connecting data ram
    logic                data_req;
    logic                data_write;
    logic [`XLEN/8-1:0]  data_wstrb;
    logic [15:0]         data_addr;
    logic [`XLEN-1:0]    data_wdata;
    logic                data_addr_ok;
    logic                data_data_ok;
    logic [`XLEN-1:0]    data_rdata;

    // bus connecting gpio
    logic                gpio_req;
    logic                gpio_write;
    logic [`XLEN/8-1:0]  gpio_wstrb;
    logic [15:0]         gpio_addr;
    logic [`XLEN-1:0]    gpio_wdata;
    logic                gpio_addr_ok;
    logic                gpio_data_ok;
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
    assign gpio_addr  = dram_addr[15:0];
    assign gpio_wdata = dram_wdata;

    assign data_req   = dram_req & decode_is_dram;
    assign data_write = dram_write;
    assign data_wstrb = dram_wstrb;
    assign data_addr  = dram_addr[15:0];
    assign data_wdata = dram_wdata;

    assign dram_addr_ok = (decode_is_gpio & gpio_addr_ok) | (decode_is_dram & data_addr_ok);
    // For simplicity, we just OR the data_ok assuming the following conditions hold true:
    //  1.) data_ok is onehot and only one target receives the request each time.
    //  2.) target device should only drive data_ok to high whenever its data is ready.
    assign dram_data_ok = gpio_data_ok | data_data_ok;
    assign dram_rdata = ({`XLEN{gpio_data_ok}} & gpio_rdata) |
                        ({`XLEN{dram_data_ok}} & data_rdata) ;

    // --------------------------------------
    // Module Instantiation
    // --------------------------------------

    core u_core (.*);

    // create a 4KB RAM to hold both the instruction and data
    ram  #(.AW(10), .DW(`XLEN))
    memory
    (
        .clk(clk),
        .instr_req(iram_req),
        .instr_write(iram_write),
        .instr_wstrb(iram_wstrb),
        .instr_addr(iram_addr[11:2]),
        .instr_wdata(iram_wdata),
        .instr_addr_ok(iram_addr_ok),
        .instr_data_ok(iram_data_ok),
        .instr_rdata(iram_rdata),
        .data_req(data_req),
        .data_write(data_write),
        .data_wstrb(data_wstrb),
        .data_addr(data_addr[11:2]),
        .data_wdata(data_wdata),
        .data_addr_ok(data_addr_ok),
        .data_data_ok(data_data_ok),
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
        .gpio_addr_ok(gpio_addr_ok),
        .gpio_data_ok(gpio_data_ok),
        .gpio_rdata(gpio_rdata)
    );

endmodule
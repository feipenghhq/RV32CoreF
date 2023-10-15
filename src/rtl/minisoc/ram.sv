/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/30/2023
 *
 * ------------------------------------------------------------------------------------------------
 * ram: A 2 RW port memory
 * ------------------------------------------------------------------------------------------------
 */

`timescale 1ns/10ps

module ram #(
    parameter AW = 12,  // address width
    parameter DW = 32   // data width
) (
    input  logic                clk,

    // instruction ports
    input  logic                instr_req,
    input  logic                instr_write,
    input  logic [DW/8-1:0]     instr_wstrb,
    input  logic [AW-1:0]       instr_addr,     // word address
    input  logic [DW-1:0]       instr_wdata,
    output logic                instr_ready,
    output logic                instr_rvalid,
    output logic [DW-1:0]       instr_rdata,
    // data ports
    input  logic                data_req,
    input  logic                data_write,
    input  logic [DW/8-1:0]     data_wstrb,
    input  logic [AW-1:0]       data_addr,      // word address
    input  logic [DW-1:0]       data_wdata,
    output logic                data_ready,
    output logic                data_rvalid,
    output logic [DW-1:0]       data_rdata
);

    parameter NUM_BYTE = DW/8;
    parameter DEPTH = 2 ** AW;

    reg [DW-1:0] mem[0:DEPTH-1];

    // ---------------------------------------------
    // address ports
    // ---------------------------------------------

    initial begin
        instr_rvalid = 1'b0;
        instr_ready = 1'b0;
    end

    always @(posedge clk) begin
        if ($test$plusargs("IRAM_RANDOM_rvalid")) begin
            instr_rdata <= #10ns mem[instr_addr];
            instr_rvalid <= #10ns instr_req & ~instr_write & instr_ready;
        end
        else begin
            instr_rdata <= mem[instr_addr];
            instr_rvalid <= instr_req & ~instr_write & instr_ready;
        end
    end

    always @(instr_req) begin
        if ($test$plusargs("IRAM_RANDOM_ready")) begin
            instr_ready = 1'b0;
            if (instr_req) begin
                repeat ($urandom_range(3)) @(posedge clk);
                #0 instr_ready = 1'b1;
            end
        end
        else begin
            instr_ready = 1'b1;
        end
    end

    // ---------------------------------------------
    // data ports
    // ---------------------------------------------

    always @(posedge clk) begin
        if ($test$plusargs("DRAM_RANDOM_rvalid")) begin
            data_rdata <= #10ns mem[data_addr];
            data_rvalid <= #10ns data_req & ~data_write & data_ready;
        end
        else begin
            data_rdata <= mem[data_addr];
            data_rvalid <= data_req & ~data_write & data_ready;
        end
    end

    genvar j;
    generate
        for (j = 0; j < NUM_BYTE; j++) begin: data_byte_enable
            always @(posedge clk) begin
                if (data_req & data_write & data_wstrb[j])
                mem[data_addr[AW-1:0]][(j+1)*8-1:j*8] <= data_wdata[(j+1)*8-1:j*8];
            end
        end
    endgenerate

    always @(data_req) begin
        if ($test$plusargs("DRAM_RANDOM_ready")) begin
            data_ready = 1'b0;
            if (data_req) begin
                repeat ($urandom_range(3)) @(posedge clk);
                #0 data_ready = 1'b1;
            end
        end
        else begin
            data_ready = 1'b1;
        end
    end

endmodule
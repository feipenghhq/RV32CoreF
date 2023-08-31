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

module ram #(
    parameter AW = 32,  // address width
    parameter DW = 32   // data width
) (
    input  logic                clk,

    // instruction ports
    input  logic                instr_req,
    input  logic                instr_write,
    input  logic [DW/8-1:0]     instr_wstrb,
    input  logic [AW-1:0]       instr_addr,
    input  logic [DW-1:0]       instr_wdata,
    output logic                instr_addr_ok,
    output logic                instr_data_ok,
    output logic [DW-1:0]       instr_rdata,
    // data ports
    input  logic                data_req,
    input  logic                data_write,
    input  logic [DW/8-1:0]     data_wstrb,
    input  logic [AW-1:0]       data_addr,
    input  logic [DW-1:0]       data_wdata,
    output logic                data_addr_ok,
    output logic                data_data_ok,
    output logic [DW-1:0]       data_rdata
);

    parameter DEPTH = 2 ** AW;
    parameter NUM_BYTE = DW/8;

    reg [DW-1:0] mem[DEPTH-1:0];

    // address ports
    always @(posedge clk) begin
        instr_data_ok <= instr_req & instr_write;
        instr_rdata <= mem[instr_addr];
    end

    assign instr_addr_ok = 1'b1; // always accept request

    // data ports
    always @(posedge clk) begin
        data_data_ok <= data_req & data_write;
        data_rdata <= mem[data_addr];
    end

    genvar j;
    generate
        for (j = 0; j < NUM_BYTE; j++) begin: data_byte_enable
            always @(posedge clk) begin
                if (data_req & data_write & data_wstrb[j])
                mem[data_addr][(j+1)*8-1:j*8] <= data_wdata[(j+1)*8-1:j*8];
            end
        end
    endgenerate

    assign data_addr_ok = 1'b1; // always accept request

endmodule
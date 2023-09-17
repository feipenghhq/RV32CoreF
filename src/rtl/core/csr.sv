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
`include "riscv_isa.svh"

// macro for defining CSR register and generate write/read logic
`ifndef CSR_REG_LOGIC
`define CSR_REG_LOGIC(name, id) \
logic  ``name``_ctrl_hit; \
logic  ``name``_ctrl_read; \
logic  ``name``_ctrl_write; \
assign ``name``_ctrl_hit   = (csr_addr == id); \
assign ``name``_ctrl_read  = ``name``_ctrl_hit & csr_read; \
assign ``name``_ctrl_write = ``name``_ctrl_hit & csr_wen;
`endif

// macro for defining a CSR field and generate write/read logic (SW read/write)
// 1. csr instruction access
`ifndef CSR_FIELD_LOGIC_SW
`define CSR_FIELD_LOGIC_SW(csr, field, width, rst_val, range) \
logic [width-1:0] ``csr``_``field``; \
always @(posedge clk) begin \
    if (!rst_b) ``csr``_``field`` <= rst_val; \
    else begin \
        if (``csr``_ctrl_write) csr``_``field`` <= csr_wdata``range``; \
    end \
end
`endif

// macro for defining a CSR field and generate write/read logic (HW write & SW read/write)
// 1. hardware write value to the csr field
// 2. csr instruction access
`ifndef CSR_FIELD_LOGIC_HW
`define CSR_FIELD_LOGIC_HW(csr, field, width, rst_val, wen, range) \
logic [width-1:0] ``csr``_``field``; \
always @(posedge clk) begin \
    if (!rst_b) ``csr``_``field`` <= rst_val; \
    else begin \
        if (wen) ``csr``_``field`` <= csr_wr_``csr``_``field``; \
        else if (``csr``_ctrl_write) csr``_``field`` <= csr_wdata``range; \
    end \
end
`endif

// macro for defining a CSR field and generate write/read logic (HW set & SW set/clear)
// 1. hardware set the the csr field to one
// 2. csr instruction access (write 0 to clear or write 1 to set)
`ifndef CSR_FIELD_LOGIC_SC
`define CSR_FIELD_LOGIC_SC(csr, field, width, rst_val, range) \
logic [width-1:0] ``csr``_``field``; \
always @(posedge clk) begin \
    if (!rst_b) ``csr``_``field`` <= rst_val; \
    else begin \
        if (csr_set_``csr``_``field``) ``csr``_``field`` <= 1'b1; \
        else if (``csr``_ctrl_write) csr``_``field`` <= csr_wdata``range; \
    end \
end
`endif

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
    output logic [`XLEN-1:0]    csr_read_data,
    // CSR field HW write
    input  logic                csr_wr_mstatus_mie,
    input  logic                csr_wr_mstatus_mpie,
    input  logic [`XLEN-1:0]    csr_wr_mepc_mepc,
    input  logic [`XLEN-1:0]    csr_wr_mtval_mtval,
    input  logic [`XLEN-2:0]    csr_wr_mcause_exception_code,
    input  logic                csr_wr_mcause_interrupt,
    input  logic                csr_set_mip_msip,
    input  logic                csr_set_mip_mtip,
    input  logic                csr_set_mip_meip,
    // CSR field HW read
    output logic                csr_rd_mstatus_mie,
    output logic                csr_rd_mstatus_mpie,
    output logic                csr_rd_mie_msie,
    output logic                csr_rd_mie_mtie,
    output logic                csr_rd_mie_meie,
    output logic [`XLEN-3:0]    csr_rd_mtvec_base,
    output logic [1:0]          csr_rd_mtvec_mode,
    output logic [`XLEN-1:0]    csr_rd_mepc_mepc,
    // Trap related information
    input  logic                ent_trap,
    input  logic                ext_trap
);

    // ------------------------------------------
    // CSR read, write glue logic
    // ------------------------------------------

    logic [`XLEN-1:0] csr_write_data;
    logic [`XLEN-1:0] csr_set_data;
    logic [`XLEN-1:0] csr_clear_data;

    logic             csr_wen;
    logic [`XLEN-1:0] csr_wdata;

    assign csr_write_data = csr_info; // this will be rs1 value
    assign csr_set_data   = csr_info | csr_read_data;
    assign csr_clear_data = ~csr_info & csr_read_data;

    assign csr_wen = csr_write | csr_set | csr_clear;
    assign csr_wdata = ({`XLEN{csr_write}} & csr_write_data) |
                       ({`XLEN{csr_set}}   & csr_set_data)   |
                       ({`XLEN{csr_clear}} & csr_clear_data) ;

    // ------------------------------------------
    // CSR Register
    // ------------------------------------------
    // Only implement necessary CSR register, all other register returns zero

    // ------------------------------------------
    // Machine status register (mstatus)
    // ------------------------------------------
    struct packed {
        logic           sd;
        logic [7:0]     wpri_0;
        logic           tsr;
        logic           tw;
        logic           tvm;
        logic           mxr;
        logic           sum;
        logic           mprv;
        logic [1:0]     xs;
        logic [1:0]     fs;
        logic [1:0]     mpp;
        logic [1:0]     vs;
        logic           spp;
        logic           mpie;
        logic           ube;
        logic           spie;
        logic           wpri_1;
        logic           mie;
        logic           wpri_2;
        logic           sie;
        logic           wpri_3;
    } mstatus;

    // Currently we only support Machine mode so we only need to implement MIE, MPIE
    // MPP is always 2'b11 in this case
    `CSR_REG_LOGIC(mstatus, `MSTATUS)
    `CSR_FIELD_LOGIC_HW(mstatus, mie,  1, 0, (ent_trap | ext_trap), [3])
    `CSR_FIELD_LOGIC_HW(mstatus, mpie, 1, 0, (ent_trap | ext_trap), [7])
    assign csr_rd_mstatus_mie  = mstatus_mie;
    assign csr_rd_mstatus_mpie = mstatus_mpie;

    always @(*) begin
        mstatus = '0;
        mstatus.mpp = 2'b11; // mpp is always 2'b11
        mstatus.mie = mstatus_mie;
        mstatus.mpie = mstatus_mpie;
    end

    // ------------------------------------------
    // Machine interrupt-enable register (mie)
    // ------------------------------------------

    struct packed {
        logic [19:0] rsvd;
        logic        meie;
        logic        zero5;
        logic        seie;
        logic        zero4;
        logic        mtie;
        logic        zero3;
        logic        stie;
        logic        zero2;
        logic        msie;
        logic        zero1;
        logic        ssie;
        logic        zero0;
    } mie;

    // Currently we only support Machine mode so we only need to implement MSIE, MTIE, MEIE
    `CSR_REG_LOGIC(mie, `MIE)
    `CSR_FIELD_LOGIC_SW(mie, msie, 1, 0, [3])
    `CSR_FIELD_LOGIC_SW(mie, mtie, 1, 0, [7])
    `CSR_FIELD_LOGIC_SW(mie, meie, 1, 0, [11])
    assign csr_rd_mie_msie  = mie_msie;
    assign csr_rd_mie_mtie  = mie_mtie;
    assign csr_rd_mie_meie  = mie_meie;

    always @(*) begin
        mie = '0;
        mie.msie = mie_msie;
        mie.mtie = mie_mtie;
        mie.meie = mie_mtie;
    end

    // ------------------------------------------
    // Machine ent_trap-handler base address (mtvec)
    // ------------------------------------------

    struct packed {
        logic [`XLEN-1:2]   base;
        logic [1:0]         mode;
    } mtvec;

    `CSR_REG_LOGIC(mtvec, `MTVEC)
    `CSR_FIELD_LOGIC_SW(mtvec, base, (`XLEN-2), 0, [`XLEN-1:2])
    `CSR_FIELD_LOGIC_SW(mtvec, mode, 2, 0, [1:0])
    assign csr_rd_mtvec_base = mtvec_base;
    assign csr_rd_mtvec_mode = mtvec_mode;

    assign mtvec.base = mtvec_base;
    assign mtvec.mode = mtvec_mode;

    // ------------------------------------------
    // Machine Scratch Register (mscratch)
    // ------------------------------------------

    struct packed {
        logic [`XLEN-1:0] mscratch;
    } mscratch;

    `CSR_REG_LOGIC(mscratch, `MSCRATCH)
    `CSR_FIELD_LOGIC_SW(mscratch, mscratch, `XLEN, 0, [`XLEN-1:0])

    assign mscratch.mscratch = mscratch_mscratch;

    // ------------------------------------------
    // Machine Cause Register (mcause)
    // ------------------------------------------

    struct packed {
        logic               interrupt;
        logic [`XLEN-2:0]   exception_code;
    } mcause;

    `CSR_REG_LOGIC(mcause, `MCAUSE)
    `CSR_FIELD_LOGIC_HW(mcause, interrupt, 1, 0, ent_trap, [`XLEN-1])
    `CSR_FIELD_LOGIC_HW(mcause, exception_code, (`XLEN-1), 0, ent_trap, [`XLEN-2:0])

    // ------------------------------------------
    // Machine exception program counter register (mepc)
    // ------------------------------------------

    struct packed {
        logic [`XLEN-1:0] mepc;
    } mepc;

    `CSR_REG_LOGIC(mepc, `MEPC)
    `CSR_FIELD_LOGIC_HW(mepc, mepc, (`XLEN), 0, ent_trap, [`XLEN-1:0])
    assign csr_rd_mepc_mepc = mepc_mepc;
    assign mepc.mepc = mepc_mepc;

    // ------------------------------------------
    // Machine ent_trap cause (mtval)
    // ------------------------------------------

    struct packed {
        logic [`XLEN-1:0] mtval;
    } mtval;

    `CSR_REG_LOGIC(mtval, `MTVAL)
    `CSR_FIELD_LOGIC_HW(mtval, mtval, (`XLEN), 0, ent_trap, [`XLEN-1:0])

    assign mtval.mtval = mtval_mtval;

    // ------------------------------------------
    // Machine interrupt pending (mip)
    // ------------------------------------------

    struct packed {
        logic [19:0] rsvd;
        logic        meip;
        logic        zero5;
        logic        seip;
        logic        zero4;
        logic        mtip;
        logic        zero3;
        logic        stip;
        logic        zero2;
        logic        msip;
        logic        zero1;
        logic        ssip;
        logic        zero0;
    } mip;

    // Currently we only support Machine mode so we only need to implement MSIE, MTIE, MEIE
    `CSR_REG_LOGIC(mip, `MIP)
    `CSR_FIELD_LOGIC_SC(mip, msip, 1, 0, [3])
    `CSR_FIELD_LOGIC_SC(mip, mtip, 1, 0, [7])
    `CSR_FIELD_LOGIC_SC(mip, meip, 1, 0, [11])

    always @(*) begin
        mip = '0;
        mip.msip = mip_msip;
        mip.mtip = mip_mtip;
        mip.meip = mip_mtip;
    end


    // ------------------------------------------
    // Final CSR read decode
    // ------------------------------------------

    assign csr_read_data =  ({`XLEN{mstatus_ctrl_read}}  & mstatus)  |
                            ({`XLEN{mie_ctrl_read}}      & mie)      |
                            ({`XLEN{mtvec_ctrl_read}}    & mtvec)    |
                            ({`XLEN{mscratch_ctrl_read}} & mscratch) |
                            ({`XLEN{mcause_ctrl_read}}   & mcause)   |
                            ({`XLEN{mepc_ctrl_read}}     & mepc)     |
                            ({`XLEN{mip_ctrl_read}}      & mip)      ;

endmodule
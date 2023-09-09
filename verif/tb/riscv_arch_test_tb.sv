/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 09/05/2023
 *
 * ------------------------------------------------------------------------------------------------
 * riscv_arch_test_tb: Testbench for riscv_arch_test
 * ------------------------------------------------------------------------------------------------
 */

`timescale 1ns/10ps

`define RAM_PATH u_minisoc.memory.mem
`define REG_PATH u_minisoc.u_core.u_id.u_regfile.register
`define PC       u_minisoc.u_core.u_if.pc

`define BEGIN_SIGNATURE_PTR 16'h3FF0
`define END_SIGNATURE_PTR 16'h3FF4
`define BEGIN_SIGNATURE `RAM_PATH[`BEGIN_SIGNATURE_PTR/4]
`define END_SIGNATURE `RAM_PATH[`END_SIGNATURE_PTR/4]

`define GOLDEN_MEM_AW 10

module tb();

    localparam GOLDEN_MEM_DEPTH = 1 << `GOLDEN_MEM_AW;

    logic       clk;
    logic       rst_b;
    logic [7:0] GPIO;
    string      test_name;
    logic [31:0] golden_mem[0:GOLDEN_MEM_DEPTH-1];

    // ------------------------------
    // DUT
    // ------------------------------
    localparam RAM_AW = 22;
    minisoc #(RAM_AW) u_minisoc (.*);

    // ------------------------------
    // Drive clock and reset
    // ------------------------------
    // clocks
    initial clk = 0;
    always #5 clk = ~clk;
    // rst_b
    initial begin
        rst_b = 1'b0;
        repeat (5) @(posedge clk);
        #0 rst_b = 1'b1;
    end

    // ------------------------------
    // Load the memory
    // ------------------------------
    // read memory data (including both instruction and data) and golden reference data
    initial begin
        $readmemh("memory.data", `RAM_PATH);
        $readmemh("golden.reference_output", golden_mem);
    end

    // ------------------------------
    // Run test and check if the test passed or failed
    // ------------------------------

    initial begin
        fork
            test_timeout();
            check_test();
        join_any
    end

    task check_test;
        logic [31:0]    begin_signature;
        logic [31:0]    end_signature;
        logic           pass;
        integer         failed_test;
        while (1) begin
            #1000; // Check the result every 1000 ns
            begin_signature = `BEGIN_SIGNATURE;
            end_signature = `END_SIGNATURE;
            if ((end_signature > begin_signature) && (begin_signature > 16)) begin
                //$info("TEST FINIISHED.");
                compare_golden(begin_signature, end_signature, pass, failed_test);
                if (pass) begin
                    $info("TEST RESULT: PASS. TEST NAME: %s", test_name);
                    #100;
                    $finish;
                end
                else if (!pass) begin
                    $fatal(1, "TEST RESULT: FAIL. TEST NAME: %s\nFailed Test Case: %0d", test_name, failed_test);
                    #100;
                end
                #100;
                $finish;
            end
        end
    endtask

    task compare_golden;
        input [31:0]    begin_signature;
        input [31:0]    end_signature;
        output          pass;
        output integer  failed_test;
        integer      idx;
        integer      addr;
        logic [31:0] data;

        addr = begin_signature;
        pass = 1;
        while (idx < end_signature) begin
            idx = addr / 4;
            data = `RAM_PATH[idx];
            if (data != golden_mem[idx]) begin
                pass = 0;
                failed_test = idx;
            end
            //$display("%h", data);
            addr = addr + 4;
        end
    endtask

    task test_timeout;
        #200000;
        $fatal(2, "TEST TIMEOUT. TEST NAME: %s", test_name);
    endtask

    initial begin
        if ($test$plusargs("DUMP")) begin
            $dumpfile("dump.vcd");
            $dumpvars(0,tb);
        end
    end

    initial begin
        if ($value$plusargs("TEST_NAME=%s", test_name));
    end

endmodule
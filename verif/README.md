# README

[TOC]

## Introduction

This directory contains test bench and different tests 

```
.
├── tb					-- contains the verilog testbench
└── tests				-- contains different tests
    └── riscv-tests
```



## How to run test

To compile the rtl and testbench for a test suite (such as riscv-tests):

```shell
make compile_<test_suite>

# Supported test suite
make compile_riscv_tests
```

To run all the tests in test suite

```shell
make run_<test_suite>
 
# Supported test suite
make run_riscv_tests
```

To run a specific test case

```shell
make <test_name>

# For example
make rv32ui-p-xor.verilog

# To dump the waveform
make rv32ui-p-xor.verilog WAVE=1
```

To see the manual for the makefile

```shell
make help
```



## riscv-tests

This is the riscv test taken from the following repo: <https://github.com/riscv-software-src/riscv-tests>

Modification made on the original repo:

1. File copied from original repo to riscv-tests folder

   ```
   // Test environment
   riscv-tests/isa/Makefile 		-> tests/riscv-tests/Makefile
   riscv-tests/env/p/riscv-tests.h -> tests/riscv-tests/riscv-tests.h
   riscv-tests/env/p/link.ld 		-> tests/riscv-tests/link.ld
   /riscv-tests/isa/macros/scalar/test_macros.h -> tests/riscv-tests/test_macros.h
   
   // Test case
   riscv-tests/isa/rv64ui 			-> tests/riscv-tests/rv32ui
   ```

2. Updated the original Makefile and the Makefrag file in rv32ui folder
3. Removed all the rv64 related instruction from the rv32ui folder
4. Modified riscv_test.h
   1. Updated RVTEST_CODE_BEGIN macro
   2. Updated RVTEST_PASS and RVTEST_FAIL macro

5. To compile and generate the test case

   ```shell
   cd tests/riscv-tests
   make
   ```

   

   

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

### Modification made on the original repo

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
3. Update the link.ld file.
4. Removed all the rv64 related instruction from the rv32ui folder
5. Modified riscv_test.h
   1. Updated RVTEST_CODE_BEGIN macro
   2. Updated RVTEST_PASS and RVTEST_FAIL macro

   Please check the file itself to see details.

6. To compile and generate the test case

   ```shell
   cd tests/riscv-tests
   make
   ```




## riscv-arch-tests

This is the riscv arch test taken from the following repo: https://github.com/riscv-non-isa/riscv-arch-test/tree/old-framework-2.x

The main branch used a newer flow involving using other tools/flow so we used the old framework

### Modification made on the original repo

The compile flow in riscv-arch-tests is also complex so we reuse the same compilation flow used in riscv-tests and only copied the tests related files from riscv-arch-tests repo. All the other supporting files are created by ourselves.

1. File copied from original repo to riscv-arch-tests folder

   ```text
   // Test case and environment
   riscv-arch-test/riscv-test-suite/rv32i_m -> tests/riscv-arch-test/riscv-test-suite/rv32i_m
   riscv-arch-test/riscv-test-suite/env/* -> tests/riscv-arch-test
   riscv-arch-test/riscv-test-env/p/riscv_test.h -> tests/riscv-arch-test/riscv_test.h
   
   // Test case
   ```

2. Added link.ld file. The file is based on link.ld file in riscv-tests

3. Added model_test.h file. The file is based on <https://github.com/riscv-non-isa/riscv-arch-test/blob/old-framework-2.x/riscv-target/rocket/model_test.h>

4. Updated riscv_test.h

   ```
   #include "../encoding.h" => #include "encoding.h"
   ```

   

5. Added a Makefile to compile and generate verilog dump. The Makefile is based on riscv-tests
6. To compile and generate the test case

```shell
cd tests/riscv-tests
make
```




# README

[TOC]

## Introduction

This directory contains test bench and different tests 

```
.
├── tb						# Cntains the verilog testbench
├── scripts					# Contains some useful scripts
└── tests					# Cntains different tests
    ├── riscv-arch-test		# Contains riscv-arch-test code
    └── riscv-tests    		# Contains riscv-tests code
```



## How to run test

**Print the manual for the makefile**

```shell
make help
```

**Run everything in one shot**

```shell
make all
```

**Run each test suite for individual test**

1. Compile the rtl and testbench for a test suite (such as riscv-tests):

```shell
make compile_<test_suite>

# Supported test suite
make compile_riscv_tests
make compile_riscv_arch_tests
```

2. run all the tests in test suite

```shell
make run_<test_suite>
 
# Supported test suite
make run_riscv_tests
make run_riscv_arch_tests
```

3. run a specific test case

```shell
make <test_name>

# For example
make rv32ui-p-xor

# To dump the waveform
make rv32ui-p-xor WAVE=1
```



## riscv-tests

This is the riscv test taken from the following repo: <https://github.com/riscv-software-src/riscv-tests>

Compile and generate the test case:

```shell
cd tests/riscv-tests
make
```

### Modification made on the original repo

1. File copied from original repo to riscv-tests folder

   ```
   // Test environment
   riscv-tests/isa/Makefile 					-> tests/riscv-tests/Makefile
   riscv-tests/env/p/riscv-tests.h 			-> tests/riscv-tests/riscv-tests.h
   riscv-tests/env/p/link.ld 					-> tests/riscv-tests/link.ld
   riscv-tests/isa/macros/scalar/test_macros.h -> tests/riscv-tests/test_macros.h
   
   // Test case: Copied the following test case
   rv32ui/rv64ui
   	- Removed ma_data.S test because misaligned access not supported in HW
   
   ```

2. Updated the original Makefile

3. Update the link.ld file.

5. Modified riscv_test.h
   1. Updated RVTEST_CODE_BEGIN macro
   2. Updated RVTEST_PASS and RVTEST_FAIL macro

   



## riscv-arch-tests

This is the riscv arch test taken from the following repo: https://github.com/riscv-non-isa/riscv-arch-test/tree/old-framework-2.x

The main branch used a newer flow involving using other tools/flow so we used the old framework.

Compile and generate the test case

```shell
cd tests/riscv-tests
make
```

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

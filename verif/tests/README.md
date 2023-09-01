# README

## riscv-tests

This is the riscv test taken from the following repo: <https://github.com/riscv-software-src/riscv-tests>

Modification made on the original repo:

1. File copied from original repo to riscv-tests folder

   ```
   // Test environment
   riscv-tests/isa/Makefile 		-> riscv-tests/Makefile
   riscv-tests/env/p/riscv-tests.h -> riscv-tests/riscv-tests.h
   riscv-tests/env/p/link.ld 		-> riscv-tests/link.ld
   /riscv-tests/isa/macros/scalar/test_macros.h -> riscv-tests/test_macros.h
   
   // Test case
   riscv-tests/isa/rv64ui 			-> riscv-tests/rv32ui
   ```

2. Updated the original Makefile and the Makefrag file in rv32ui folder
3. Removed all the rv64 related instruction from the rv32ui folder
4. Modified riscv_test.h
   1. Updated RVTEST_CODE_BEGIN macro
   2. Updated RVTEST_PASS and RVTEST_FAIL macro

To compile the riscv-tests

#--------------------------------------------------------------------
# Makefile for running the simulation
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# Directory
#--------------------------------------------------------------------

REPO := $(shell git rev-parse --show-toplevel)
TB_PATH := $(REPO)/verif/tb

RISCV_TESTS_PATH := $(REPO)/verif/tests/riscv-tests/generated
RISCV_TESTS_FILES := $(notdir $(wildcard $(RISCV_TESTS_PATH)/*.verilog))

include $(REPO)/src/rtl/rtl.mk
IVERILOG_OPTIONS := -g2005-sv

ifeq ($(WAVE),1)
	DUMP := DUMP
endif

#--------------------------------------------------------------------
# Command
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# help command

help:
	@echo "Usage: make <test suite/test case> [option]"
	@echo ""
	@echo "Options:"
	@echo "    WAVE=1        Dump the waveform"
	@echo ""
	@echo "Supported test suite:"
	@echo "    riscv_tests"
	@echo ""
	@echo "Example:"
	@echo "  1. Run all test case in riscv_tests:   make run_riscv_tests"
	@echo "  2. Run one test case:                  make rv32ui-p-xor.verilog"
	@echo "  3. Run one test and dump the waveform: make rv32ui-p-xor.verilog WAVE=1"


#--------------------------------------------------------------------
# RISCV TESTS

riscv_tests = $(RISCV_TESTS_FILES)

run_riscv_tests: riscv_tests_pre $(riscv_tests) riscv_tests_summary

# Print out the summary of the tests
riscv_tests_summary:
	@echo ""
	@echo "-------- Test Summary --------"
	@echo "Number of passed test:" `grep PASS riscv_tests.log -c`
	@echo "Number of failed test:" `grep FAIL riscv_tests.log -c`
	@echo "Number of timeout test:" `grep "TIMEOUT" riscv_tests.log -c`
	@echo "Summary of failed test:" `grep -B 1 FAIL riscv_tests.log | grep "\w*\-\w*\-\w*.verilog" -o`
	@echo "Summary of timeout test:" `grep -B 1 TIMEOUT riscv_tests.log | grep "\w*\-\w*\-\w*.verilog" -o`

# clean up the log files and the memory files before running simulation
riscv_tests_pre:
	@rm -f memory.data
	@rm -f riscv_tests.log
	@echo "-------- Running RISCV TESTS --------"

# Generate the memory contant and run the test
$(riscv_tests): compile_riscv_tests
	@echo "Running test: $@" | tee -a riscv_tests.log
	@python3 gen_instr_data.py $(RISCV_TESTS_PATH)/$@ 1024
	@-vvp run_riscv_tests +$(DUMP) | tee -a riscv_tests.log
	@rm -f memory.data

# Compile the source RTL with testbench RTL
compile_riscv_tests: $(SV_FILE) $(TB_PATH)/riscv_tests_tb.sv
	@iverilog $(IVERILOG_OPTIONS) -I $(INC_DIR) $(SV_FILE) $(TB_PATH)/riscv_tests_tb.sv -o run_riscv_tests

#--------------------------------------------------------------------
# Compile RTL only

rtl_compile:
	@iverilog $(IVERILOG_OPTIONS) -I $(INC_DIR) $(SV_FILE) -o run_rvcore_f

rtl_lint:
	verilator -lint-only -Wall $(SV_FILE) -I$(INC_DIR)

#--------------------------------------------------------------------
# Clean target

clean:
	rm -rf run_* *.vcd *.log *.data
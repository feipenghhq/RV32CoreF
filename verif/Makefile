#--------------------------------------------------------------------
# Makefile for running the simulation
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# Directory
#--------------------------------------------------------------------

REPO := $(shell git rev-parse --show-toplevel)
TB_PATH := $(REPO)/verif/tb
OUTPUT := output

RISCV_TESTS_PATH := $(REPO)/verif/tests/riscv-tests/generated
RISCV_TESTS_TEST := $(basename $(notdir $(wildcard $(RISCV_TESTS_PATH)/*.verilog))) # get rid of .verilog
RISCV_ARCH_TEST_PATH := $(REPO)/verif/tests/riscv-arch-test/generated
RISCV_ARCH_TEST_TEST := $(basename $(basename $(notdir $(wildcard $(RISCV_ARCH_TEST_PATH)/*.elf.verilog)))) # get rid of .elf.verilog

#--------------------------------------------------------------------
# Options
#--------------------------------------------------------------------

include $(REPO)/src/rtl/rtl.mk

IVERILOG_OPTIONS := -g2005-sv

ifeq ($(WAVE),1)
	DUMP := DUMP
endif

#--------------------------------------------------------------------
# functions
#--------------------------------------------------------------------

# compile template for each test
define func_template

run_$(1): compile_$(1) $(1)_pre $$($(1)) $(1)_summary

compile_$(1): $(SV_FILE) $(TB_PATH)/$(1)_tb.sv $(OUTPUT)
	@iverilog $(IVERILOG_OPTIONS) -I $(INC_DIR) $(SV_FILE) $(TB_PATH)/$(1)_tb.sv -o $(OUTPUT)/run_$(1)

$(eval $(call func_prepare,$(1)))

$(eval $(call func_summary,$(1),$(1).log,$(1).summary))

endef

# function for preparing
# prepare <name>
define func_prepare
$(1)_pre:
	@rm -f memory.data
	@rm -f golden.reference
	@rm -f $(OUTPUT)/$(1).log
endef

# function for running test
# func_run_test <test suite name> <test path> <test ram width>
define func_run_test
$$($(1)):
	@rm -f memory.data golden.reference_output
	@echo "Running test: $$@" | tee -a $(OUTPUT)/$(1).log
	@python3 scripts/gen_instr_data.py `find -name $$@.*verilog` '1<<$(3)'
ifeq ($(1), riscv_arch_test)
	@ln -s `find ./tests/riscv-arch-test -name $$@.reference_output` golden.reference_output
endif
	@vvp $(OUTPUT)/run_$(1) +$(DUMP) +TEST_NAME=$$@ | tee -a $(OUTPUT)/$(1).log
endef

# function for print out summary
# summary <name> <search file pattern> <output log file>
define func_summary
$(1)_summary:
	./scripts/gen_summary.sh $(1) $(OUTPUT)/$(2) $(OUTPUT)/$(3).log
endef

#--------------------------------------------------------------------
# Command
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# help command

help:
	@echo "Usage: make <test suite/test case> [option]"
	@echo ""
	@echo "Options:"
	@echo "  WAVE=1        Dump the waveform"
	@echo ""
	@echo "Test Suite:"
	@echo "  riscv_tests:       make run_riscv_tests"
	@echo "  riscv_arch_test:   make run_riscv_arch_tests"
	@echo ""
	@echo "Run individual test case:"
	@echo "  make rv32ui-p-xor"
	@echo ""
	@echo "Run individual test case and dump waveform:"
	@echo "  make rv32ui-p-xor WAVE=1"

show_riscv_tests:
	@echo $(RISCV_TESTS_TEST)

show_riscv_arch_test:
	@echo $(RISCV_ARCH_TEST_TEST)

$(OUTPUT):
	mkdir $(OUTPUT)

#--------------------------------------------------------------------
# all tests

all: run_riscv_tests run_riscv_arch_test all_test_summary

$(eval $(call func_summary,all_test,"*.log",summary))

#--------------------------------------------------------------------
# RISCV ARCH TESTS

vpath %.elf.verilog $(RISCV_ARCH_TEST_PATH)
riscv_arch_test = $(RISCV_ARCH_TEST_TEST)
$(eval $(call func_template,riscv_arch_test))
$(eval $(call func_run_test,riscv_arch_test,$(RISCV_ARCH_TEST_PATH),24))

#--------------------------------------------------------------------
# RISCV TESTS
vpath %.verilog $(RISCV_TESTS_PATH)
riscv_tests = $(RISCV_TESTS_TEST)
$(eval $(call func_template,riscv_tests))
$(eval $(call func_run_test,riscv_tests,$(RISCV_TESTS_PATH),12))

#--------------------------------------------------------------------
# Compile RTL only

rtl_compile: $(OUTPUT)
	@iverilog $(IVERILOG_OPTIONS) -I $(INC_DIR) $(SV_FILE) -o $(OUTPUT)/rvcore_f

rtl_lint:
	@verilator -lint-only -Wall $(SV_FILE) -I$(INC_DIR)

#--------------------------------------------------------------------
# Clean target

clean:
	rm -rf $(OUTPUT) *.vcd *.data golden.reference_output

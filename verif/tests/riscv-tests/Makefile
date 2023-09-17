#=======================================================================
# Makefile for riscv-tests/isa
#-----------------------------------------------------------------------

XLEN ?= 32

src_dir := .

include $(src_dir)/rv32ui/Makefrag
include $(src_dir)/rv32mi/Makefrag

default: all

#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

RISCV_PREFIX ?= riscv-none-embed-
RISCV_GCC ?= $(RISCV_PREFIX)gcc
RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles
RISCV_OBJDUMP ?= $(RISCV_PREFIX)objdump --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data
RISCV_OBJCOPY ?= $(RISCV_PREFIX)objcopy

# define search path for %.S file
vpath %.S $(src_dir)

#------------------------------------------------------------
# Build assembly tests

define compile_template

$$($(1)_p_tests): $(1)-p-%: $(1)/%.S
	mkdir -p generated
	$$(RISCV_GCC) $(2) $$(RISCV_GCC_OPTS) -I$(src_dir) -T$(src_dir)/link.ld $$< -o generated/$$@
$(1)_tests += $$($(1)_p_tests)

$(1)_tests_dump = $$(addsuffix .dump, $$($(1)_tests))

$(1): $$($(1)_tests_dump)

.PHONY: $(1)

tests += $$($(1)_tests)

endef

$(eval $(call compile_template,rv32ui,-march=rv32i -mabi=ilp32))
$(eval $(call compile_template,rv32mi,-march=rv32i -mabi=ilp32))

tests_dump = $(addsuffix .dump, $(tests))
tests_hex = $(addsuffix .hex, $(tests))

#------------------------------------------------------------
# Default

all: $(tests_dump)

# Note:
# --verilog-data-width=4 makes the each data 4 bytes since my memory model read 4 bytes at a time
# --reverse-bytes=4 is to make the word in the verilog file little endiness
# $(RISCV_OBJCOPY) --reverse-bytes=4 --verilog-data-width 4 -O verilog generated/$< generated/$<.verilog_reversed_4byte

%.dump: %
	$(RISCV_OBJDUMP) generated/$< > generated/$@
	$(RISCV_OBJCOPY) --reverse-bytes=4 --verilog-data-width 4 -O verilog generated/$< generated/$<.verilog
	rm -r generated/$<

#------------------------------------------------------------
# Clean up

clean:
	rm -rf generated

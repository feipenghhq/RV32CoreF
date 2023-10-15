#--------------------------------------------------------------------
# makefile for vivado flow
#--------------------------------------------------------------------

#--------------------------------------------------------------------
# Directory

PWD		  = $(shell pwd)
REPO 	  = $(shell git rev-parse --show-toplevel)

#--------------------------------------------------------------------
# Configure variable

SCRIPT_DIR 	= $(REPO)/tools/vivado

#--------------------------------------------------------------------
# Project specific variable

# device
DEVICE  ?= xc7a35ticsg324-1L

# project name
PROJECT ?= minisoc

# top level name
TOP ?= minisoc

# verilog
include $(REPO)/src/rtl/rtl.mk
VERILOG	+= $(SV_FILE)

# include directory
SEARCH += $(REPO)/src/rtl/include

# xdc
XDC += $(PWD)/xdc/timing.xdc
XDC += $(PWD)/xdc/pin.xdc

# project output directory
OUT_DIR ?= $(PWD)/output

export VIVADO_DEVICE 	= $(DEVICE)
export VIVADO_PRJ 		= $(PROJECT)
export VIVADO_TOP    	= $(TOP)
export VIVADO_VERILOG  	= $(VERILOG)
export VIVADO_DEFINE	= $(DEFINE)
export VIVADO_SEARCH   	= $(SEARCH)
export VIVADO_XDC		= $(XDC)

#--------------------------------------------------------------------
# Commands
#--------------------------------------------------------------------



$(OUT_DIR)/$(TOP).bit: clean $(OUT_DIR)
	cd $(OUT_DIR) && vivado -mode tcl -source $(SCRIPT_DIR)/vivado_build.tcl | tee build.log

build: $(OUT_DIR)/$(TOP).bit
	cp $(OUT_DIR)/$(TOP).bit .

pgm:
	cd $(OUT_DIR) && vivado -mode tcl -source $(SCRIPT_DIR)/vivado_program.tcl | tee program.log

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

clean:
	rm -rf $(OUT_DIR)

###################################################################################################
# Makefile to compile the source RTL code
###################################################################################################

REPO := $(shell git rev-parse --show-toplevel)
include $(REPO)/src/rtl/rtl.mk

iverilog:
	@iverilog -g2005-sv -I $(INC_DIR) $(SV_FILE) -o rvcore_f

clean:
	rm -rf rvcore_f
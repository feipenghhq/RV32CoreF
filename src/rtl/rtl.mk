REPO := $(shell git rev-parse --show-toplevel)

# directories
INC_DIR  := $(REPO)/src/rtl/include
CORE_DIR := $(REPO)/src/rtl/core

# src files
INC_FILES := $(wildcard $(INC_DIR)/*.svh)
CORE_FILE := $(wildcard $(CORE_DIR)/*.sv)

SV_FILE := $(CORE_FILE)
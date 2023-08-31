REPO := $(shell git rev-parse --show-toplevel)

# directories
INC_DIR := $(REPO)/src/rtl/include
CORE_DIR := $(REPO)/src/rtl/core
MINISOC_DIR := $(REPO)/src/rtl/minisoc

# src files
INC_FILES := $(wildcard $(INC_DIR)/*.svh)
CORE_FILE := $(wildcard $(CORE_DIR)/*.sv)
MINISOC_FILE := $(wildcard $(MINISOC_DIR)/*.sv)

SV_FILE += $(CORE_FILE)
SV_FILE += $(MINISOC_FILE)
# ------------------------------------------------------------------------------------------------
# Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
#
# Project: RVCoreF
# Author: Heqing Huang
# Date Created: 09/02/2023
#
# ------------------------------------------------------------------------------------------------
# Generate the memory file for verilog from *.verilog file dump
# ------------------------------------------------------------------------------------------------
#
# This script generate the memory content for simulation from the *.iverilog
# file dump from objdump command.
#
# It reads the contents of the *.iverilog file and flattens the data in the files
# It fills up the rest of the memory with all 0
#
#
# Usage:
#   python3 gen_instr_data.py <path to verilog dump file> <memory depth>
#

import sys

INPUT = sys.argv[1]
MEMSIZE = int(eval(sys.argv[2]))
OUTPUT = "memory.data"
WORDSIZE = 4
MEM = ['00000000' for _ in range(MEMSIZE)]

idx = 0 # word index in memory

# read the *.verilog and process its contents
with open(INPUT, "r") as f:
    lines = f.readlines()
    for line in lines:
        line = line.rstrip()
        # process address line
        if "@" in line:
            addr = int(line[1:], 16)
            # the address in *.iverilog file is byte address so we need to divide it by 4 to make it word idx
            idx = int(addr / WORDSIZE)
        # process data line
        else:
            data = line.split()
            for d in data:
                MEM[idx] = d
                idx += 1

# print out the memory.data file
with open(OUTPUT, "w") as f:
    for data in MEM[0:idx]:
        f.write(data)
        f.write("\n")

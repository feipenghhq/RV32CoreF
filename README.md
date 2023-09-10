# RV32CoreF

[TOC]

## Introduction

### About the project

RV32CoreF is a RISC-V CPU Core design using Verilog/SystemVerilog.

The CPU core has the following key features:

1. Support `RV32I[M][Zicsr]` ISA (ISA in [] is optional and can be controlled by parameters).

2. 5 stage pipeline design with **IF**, **ID**, **EX**, **MEM**, **WB** stages.

3. Support 4 interrupts (external, timer, software, debug) defined in RISC-V Specification

4.  **[TBD]** Optional BPU(Branch Prediction Unit) and BTB(Branch Target Buffer) to improve branch performance.

5.  **[TBD]** Optional Cache (Both instruction and data).

For detailed micro-architecture check the following documents:

[core.md](doc/arch/core.md) for CPU core micro-architecture

### Repo Structure

```text
.
├── doc			# Document
├── src			# RTL source code
└── verif		# Verification code
```



## Getting Started

### Prerequisites

- **RISC-V Tool Chain**

  **GNU MCU Eclipse RISC-V Embedded GCC** is used to compile the C code into RISC-V ISA using newlib as the C standard library.

​		GNU MCU Eclipse RISC-V Embedded GCC: <https://gnu-mcu-eclipse.github.io/blog/2019/05/21/riscv-none-gcc-v8-2-0-2-2-20190521-released>

​		Check [riscv_tool_chain_installation](doc/riscv_tool_chain_installation.md) for details on how to install the tool chain.

- **Icarus Verilog**

​		Icarus Verilog is an open source verilog simulator. 

​		Check <http://iverilog.icarus.com/> for installation guidance.

- **Vivado**

​		Vivado are used to synthesis the design and run simulation.

​		Checks <https://www.xilinx.com/products/design-tools/vivado.html> for installation guidance.

### Running Simulation

We use riscv_tests and riscv_arch_tests to test the CPU core design.

To run simulation, check [verif/README.md](verif/README.md) for detailed instruction to run simulation.



## Acknowledgments

The CPU core architecture is mainly inspired from the book <CPU设计实战>




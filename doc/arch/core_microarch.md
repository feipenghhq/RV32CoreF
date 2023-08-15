# Core Micro-architecture

[TOC]

## 5 Stage Pipeline

This CPU core design follows the classic 5 stage pipeline design including: 

- Instruction Fetch (IF)
- Instruction Decode (ID)
- Execution (EX)
- Memory (MEM)
- Write Back (WB)



## IF Stage

IF stage contains instruction fetch unit (ifu.sv)

### instruction fetch unit (ifu.sv)

1. PC register and generation of the next PC value.

   - In normal case, PC value increase by 4 each clock cycle.

   - PC may take new value from unconditional jump (jal/jalr), or successful conditional jump (bxx)

   - PC may take new value from exception/interrupt handler

2. Generation of the control and address to access instruction RAM.

   We assume the instruction RAM use is synchronous SRAM which has one clock cycle latency. The introduction of synchronous SRAM introduced an issue that the read data is not available in the same clock cycle when we send the read request to instruction RAM.

   We have 2 solutions:

   - Solution 1: Send the RAM read request at IF stage, then the instruction is available in ID stage. **In this design, the instruction read from Instruction RAM should NOT be put into the IF/ID pipeline stage registers**

   - Solution 2: Send the RAM read request while we updating PC. **We use next PC as the read address instead of PC** so the instruction is available in IF stage.

   **In our CPU design, we use solution 2.**



## ID stage

ID stage contains instruction decoder (decoder.sv) and register file (regfile.sv). 

In ID stage, the instruction is first decoded in decoder, then the register file is accessed to grep the rs1 and rs2 value for the downstream logic. The decoded control signals, immediate value, and rs1/rs2 value are send to the pipeline stage registers.

### decoder (decoder.sv)

Decode the instruction to different control signals, and generate the immediate value.

### register file (regfile.sv)

Hold the 32 register defined in RISC-V spec. Contains 2 read port for rs1/rs2 access in ID stage and 1 write port for register value write back from WB stage.

Register 0 (r0/zero) always read 0.



## EX stage

EX stage contains Arithmetic Logic Unit (ALU, alu.sv), MUX to select different source for the ALU input, the glue logic for branch/jump control, and Data Memory Request Generation (mem_req_gen).

### Arithmetic Logic Unit (ALU, alu.sv)

ALU contains the logic for executing most of the arithmetic/logic operation in RISC-V ISA.

These are all the operations it does under RV32I Instruction Set (Floating point / Multiplication / Divide are implemented in separate logic unit if supported)

| Operation | Instruction                                       |
| --------- | ------------------------------------------------- |
| ADD       | AUIPC/LOAD/STORE/ADDI/ADD                         |
| SUB       | SUB/BEQ/BNE/BLT/BGE/BLTU/BGEU/SLT/SLTU/SLTI/SLTIU |
| XOR       | XOR/XORI                                          |
| AND       | AND/ANDI                                          |
| OR        | OR/ORI                                            |
| SLL       | SLL/SLLI                                          |
| SRL       | SRL/SRLI                                          |
| SRA       | SRA/SRAI                                          |

### ALU Operand Source

ALU has 2 input operand, they can come from different sources

Operand 1:

| Source       | Comments                                  |
| ------------ | ----------------------------------------- |
| register rs1 | R-type instruction/Store/Load/Branch/JALR |
| PC           | AUIPC/JAL                                 |

Operand 2:

| Source       | Comments                                                |
| ------------ | ------------------------------------------------------- |
| register rs2 | R-type instruction                                      |
| Immediate    | I-type instruction/Store/Load/Branch/LUI/AUIPC/JAL/JALR |

### PPA Trade off in ALU Design

We can reuse the same logic in ALU to save logic resource/gates or we can use dedicated logic for better performance and improve timing.

1. **ADD/SUB instruction** can use just one adder. For sub, use 2's complement to make it becomes add: 

   ```
   a - b = a + (-b) = a + (~b+1)
   ```

2. **LOAD/STORE instruction** can use the Adder in ALU to calculate the address since *the effective address is obtained by adding register rs1 to the sign-extended 12-bit offset.* Offset is usually stored in immediate value so the address calculation is t**he same as ADDI instruction**.

3. **SLT/SLTU/SLTI/SLTIU** instruction. Instead of directly testing if a < b, we can use the sub result with some additional logic to determine if a < b. 
4. **BEQ/BNE/BLT/BGE/BLTU/BGEU** instruction. Similarly we can use the same adder to determine the result of branch.

#### Performance related Configuration Macros

Base on the above session, we can choose to share resource to improve area or use dedicated resource to improve performance. depending on what we need (targeting area first or performance first). The following table defines some of the design choices for these options. **We will target all the optimization mentioned above first and add options to improve performance if needed**

| Macro                  | Implemented* | Description                                                  |
| ---------------------- | ------------ | ------------------------------------------------------------ |
| DEDICATED_MEM_AGU*     | No           | Use a dedicated adder to calculate data ram address in ALU   |
| DEDICATED_BRANCH_COND* | No           | Use dedicated logic to calculate the branch condition. (May not be inside ALU) |

*Implemented: Indicate if this macro and corresponding logic is implemented in the design or not.

### Data Memory Request Generation (mem_req_gen.sv)

Similar to IF stage, because we use synchronous RAM for data ram too, there are 1 read cycle latency. There are 2 options too:

- Option 1: Send the request at EX stage, data will be available in MEM stage
- Option 2: Send the request at MEM stage, data will be available in WB stage

In our CPU design, we use option 1.



## MEM Stage

Data memory read data comes back at memory stage. Memory stage contains the logic to process the memory read data.



## WB Stage

Write back stage write the data back to the register.

There are several source of the write data:

| Source        | Comment                               |
| ------------- | ------------------------------------- |
| pc + 4        | From Jal/Jalr instruction             |
| ALU output    | From arithmetic and logic instruction |
| Memory output | From Load instruction                 |
| CSR output    | From CSR instruction                  |



## Pipeline 

### Pipeline Control Signal

In ordering to simply the pipeline control such as stall and flush, we introduce a handshake control between the adjacent pipeline. Here are the signals to implement the handshake:

- `<stage>_pipe_valid`: The valid indicator for pipeline stage. This is implemented from the register. 
- `<stage>_pipe_done`: Indicate the current task on pipeline stage is completed and can be moved to the next pipeline stage
- `<stage>_pipe_ready`: Indicate that the pipeline stage is ready to take the data from previous pipeline.
- `<stage>_pipe_req`: Indicate that the pipeline request to send data into the next pipeline

Status of handshaking signals

| `<stageX>_pipe_req` | `<stageX+1>_pipe_ready` | comments                                                     |
| ------------------- | ----------------------- | ------------------------------------------------------------ |
| 0                   | 0                       | Pipeline Stalled from X+1 stage                              |
| 0                   | 1                       | Nothing from stage X to be send to stage X + 1, insert a bubble to stage X + 1 |
| 1                   | 0                       | Pipeline Stalled from X+ 1 stage                             |
| 1                   | 1                       | Successful handshake. Valid data is transferred from stage X to stage X+1 |




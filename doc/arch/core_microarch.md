# Core Micro-architecture

[TOC]

## 5 Stage Pipeline architecture

This CPU core design follows the classic 5 stage pipeline design including: 

- Instruction Fetch (IF)
- Instruction Decode (ID)
- Execution (EX)
- Memory (MEM)
- Write Back (WB)

### IF Stage

IF stage contains instruction fetch unit (ifu.sv)

#### instruction fetch unit (ifu.sv)

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



### ID stage

ID stage contains instruction decoder (decoder.sv) and register file (regfile.sv). 

In ID stage, the instruction is first decoded in decoder, then the register file is accessed to grep the rs1 and rs2 value for the downstream logic. The decoded control signals, immediate value, and rs1/rs2 value are send to the pipeline stage registers.

#### decoder (decoder.sv)

Decode the instruction to different control signals, and generate the immediate value.

We only detect illegal instruction detection on opcode. If other part of the instruction (such as funct3) is illegal, we don't detect that and the instruction might be treated as NOP depending on the encoding

#### register file (regfile.sv)

Hold the 32 register defined in RISC-V spec. Contains 2 read port for rs1/rs2 access in ID stage and 1 write port for register value write back from WB stage.

Register 0 (r0/zero) always read 0.



### EX stage

EX stage contains the following modules and functions

1. Arithmetic Logic Unit (ALU, alu.sv)
2. MUX to select different source for the ALU input
3. Branch/Jump control logic.
4. Data Memory Request Generation

#### Arithmetic Logic Unit (ALU, alu.sv)

ALU contains the logic for executing most of the arithmetic/logic operation in RISC-V ISA.

These are all the operations it does under RV32I Instruction Set (Floating point / Multiplication / Divide are implemented in separate logic unit if supported)

| Operation | Instruction                               |
| --------- | ----------------------------------------- |
| ADD       | AUIPC/LOAD/STORE/ADDI/ADD/JAL/JALR/Branch |
| SUB       | SUB/SLT/SLTU/SLTI/SLTIU                   |
| XOR       | XOR/XORI                                  |
| AND       | AND/ANDI                                  |
| OR        | OR/ORI                                    |
| SLL       | SLL/SLLI                                  |
| SRL       | SRL/SRLI                                  |
| SRA       | SRA/SRAI                                  |

#### MUX to select different source for the ALU input

ALU has 2 input signals, they can come from different sources depending on the instruction

Input Signal 1:

| Source       | Comments                                  |
| ------------ | ----------------------------------------- |
| register rs1 | R-type instruction/Store/Load/Branch/JALR |
| PC           | AUIPC/JAL/Branch                          |

Input Signal 2:

| Source       | Comments                                                |
| ------------ | ------------------------------------------------------- |
| register rs2 | R-type instruction                                      |
| Immediate    | I-type instruction/Store/Load/Branch/LUI/AUIPC/JAL/JALR |

#### Branch/Jump control logic

1. The branch/jump address calculation is performed in ALU.

2. We also need logic to check if branch are successful or not. To simplify the design, we reuse the ALU module to generate the result:

   1.  beq/bne: use ALU_OP_SUB opcode (doing subtruction) and check if the result is zero/not zero
   2. blt/bge: use ALU_OP_SLT opcode (doing SLT) and check if the result is zero/not zero
   3. bltu/bgeu: use ALU_OP_SLTU opcode (doing SLTU) and check if the result is zero/not zero

   We can tie all the other opcode to zero so that other logic will be optimized by synthesis tool

#### Data Memory Request Generation

1. The memory address calculation is performed in ALU.

2. Similar to IF stage, because we use synchronous RAM for data ram too, there are 1 read cycle latency. There are 2 options too:

   - Option 1: Send the request at EX stage, data will be available in MEM stage

   - Option 2: Send the request at MEM stage, data will be available in WB stage

​	**In our CPU design, we use option 1.**



#### Resource Reuse in ALU Design

We can reuse the same logic in ALU to save logic resource/gates or we can use dedicated logic for better performance and improve timing.

1. **ADD/SUB instruction** can use just one adder. For sub, use 2's complement to make it becomes add: 

   ```
   a - b = a + (-b) = a + (~b+1)
   ```

2. **LOAD/STORE instruction** can use the Adder in ALU to calculate the address since *the effective address is obtained by adding register rs1 to the sign-extended 12-bit offset.* Offset is usually stored in immediate value so the address calculation is t**he same as ADDI instruction**.

3. **SLT/SLTU/SLTI/SLTIU** instruction. Instead of directly testing if a < b, we can use the sub result with some additional logic to determine if a < b. 
4. **BEQ/BNE/BLT/BGE/BLTU/BGEU** instruction. We use the adder in ALU to calculate the target address.
5. **JAL/JALR** instruction. We use the adder in ALU to calculate the target address.



### MEM Stage

Data memory read data comes back at memory stage. Memory stage contains the logic to process the memory read data.



### WB Stage

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
- `<stage>_pipe_flush`: Indicate the the there is a request from previous stage to flush the current stage
- `<stage>_valid`: Indicate the the current pipeline stage is valid

Status of handshaking signals

| `<stageX>_pipe_req` | `<stageX+1>_pipe_ready` | comments                                                     |
| ------------------- | ----------------------- | ------------------------------------------------------------ |
| 0                   | 0                       | Pipeline Stalled from X+1 stage                              |
| 0                   | 1                       | Nothing from stage X to be send to stage X + 1, insert a bubble to stage X + 1 |
| 1                   | 0                       | Pipeline Stalled from X+ 1 stage                             |
| 1                   | 1                       | Successful handshake. Valid data is transferred from stage X to stage X+1 |

General logic for the control signal

(Note: This need to updated once we consider exception)

```verilog
// Assuming s1 talks to s2:

// s1_valid: pipeline itself is valid and no flusing request from next stage
assign s1_valid = s1_pipe_valid & ~s2_pipe_flush;

// s1_pipe_done: task in s1 stage has completed
assign s1_pipe_done = <task in s1 has completed>;

// s1_pipe_ready: task in s1 stage has completed (s1_pipe_done) and next stage is ready
assign s1_pipe_ready = s2_pipe_ready & s2_pipe_done;

// s1_pipe_req: task in s1 stage has completed (s1_pipe_done) and stage itself is valid (s1_valid)
assign s1_pipe_req = s1_pipe_done & s1_valid;

// s1_pipe_flush: task in s1 reequest flusing or next stage request flusing
assign s1_pipe_flush = <task in s1 request flushing> | s2_pipe_flush;
```



### Control Conflict

1. Pipeline stall due to memory access and interrupt want to flush the pipeline

   There will be an issue that if the data bus require that the request signal being asserted until the request has been accepted/completed. Then we can't flush the read/write instruction in the memory stage.

   To deal with this, let's carry the interrupt starting from the EX stage ??

2. If interrupt happens and we have a successful branch or jump instruction in the pipeline,

   then, what is the return address of the interrupt? It should be the target pc instead of PC + 4 of the branch instruction itself

## Exception and Interrupt


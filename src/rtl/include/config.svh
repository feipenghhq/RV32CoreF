/* ------------------------------------------------------------------------------------------------
 * Copyright (c) 2022. Heqing Huang (feipenghhq@gmail.com)
 *
 * Project: RVCoreF
 * Author: Heqing Huang
 * Date Created: 08/12/2023
 *
 * ------------------------------------------------------------------------------------------------
 * CPU Config
 * ------------------------------------------------------------------------------------------------
 */

`ifndef __RV32COREF_CONFIG__
`define __RV32COREF_CONFIG__

// CPU Width
`define XLEN            32

// Register Number
`define REG_NUM         32
`define REG_AW          $clog2(`REG_NUM)

`endif
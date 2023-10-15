# ----------------------------------------
# Program Device
# ----------------------------------------

set VIVADO_TOP      $::env(VIVADO_TOP)

open_hw_manager
connect_hw_server
open_hw_target
set_property PROGRAM.FILE "$VIVADO_TOP.bit" [current_hw_device]
program_hw_device [current_hw_device]

exit
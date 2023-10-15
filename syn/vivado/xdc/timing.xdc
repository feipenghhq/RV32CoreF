# Main clock - 100Mhz
create_clock -name clk -period 10.000 [get_ports clk]
set_input_jitter [get_clocks -of_objects [get_ports clk]] 0.100
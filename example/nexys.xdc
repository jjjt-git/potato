set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
set_property -dict {PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports reset_n]
set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports {trace_enable[1]}]
set_property -dict {PACKAGE_PIN V11 IOSTANDARD LVCMOS33} [get_ports {trace_enabled[1]}]
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports {trace_enable[0]}]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {trace_enabled[0]}]
set_property -dict {PACKAGE_PIN V14 IOSTANDARD LVCMOS33} [get_ports global_st]
set_property -dict {PACKAGE_PIN R11 IOSTANDARD LVCMOS33} [get_ports trace_dumping]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports trace_gather]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports uart0_txd]
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports uart0_rxd]


#set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports global_en]


#set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[0]}]
#set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {cache_st[0]}]
#set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[1]}]
#set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {cache_st[1]}]
#set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[2]}]
#set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {cache_st[2]}]
#set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[3]}]
#set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {cache_st[3]}]
#set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[4]}]
#set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {cache_st[4]}]

#set_property -dict {PACKAGE_PIN D3 IOSTANDARD LVCMOS33} [get_ports uart0_cts]
#set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports uart1_txd]
#set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports uart1_rxd]

# The Potato Processor - A simple processor for FPGAs
# (c) Kristian Klomsten Skordal 2016 <kristian.skordal@wafflemail.net>
# Report bugs and issues on <https://github.com/skordal/potato/issues>
#
# Adapted from arty.xdc for Nexys A7 100T by Jacob Tilger

# Set operating conditions to improve temperature estimation:
set_operating_conditions -airflow 0
set_operating_conditions -heatsink low

# Clock signal:
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} -add [get_ports clk]

set_property BITSTREAM.CONFIG.USR_ACCESS 0xFFFFFFFF [current_design]

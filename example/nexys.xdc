set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
set_property -dict {PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports reset_n]

set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports trace_enable]
set_property -dict {PACKAGE_PIN V11 IOSTANDARD LVCMOS33} [get_ports trace_enabled]

set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports global_en]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports global_st]

set_property -dict {PACKAGE_PIN R11 IOSTANDARD LVCMOS33} [get_ports trace_dumping]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports trace_gather]

set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[0]}]
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {cache_st[0]}]
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[1]}]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {cache_st[1]}]
set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[2]}]
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {cache_st[2]}]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[3]}]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {cache_st[3]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {cache_crtl[4]}]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {cache_st[4]}]

set_property -dict {PACKAGE_PIN D3 IOSTANDARD LVCMOS33} [get_ports uart0_cts]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports uart0_txd]
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports uart0_rxd]
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports uart1_txd]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports uart1_rxd]

# "constrain" inputs and outputs
create_clock -period 100.000 -name virual_io -waveform {0.000 50.000}
set_input_delay 0 -clock [get_clocks virtual_io] [get_ports reset_n]
set_input_delay 0 -clock [get_clocks virtual_io] [get_ports uart0_rxd]
set_output_delay 0 -clock [get_clocks virtual_io] [get_ports uart0_txd]
set_false_path -from [get_ports reset_n]
set_false_path -from [get_ports uart0_rxd]
set_false_path -to [get_ports uart0_txd]

set_input_delay 0 -clock [get_clocks virtual_io] [get_ports cache_crtl]
set_output_delay 0 -clock [get_clocks virtual_io] [get_ports cache_st]
set_input_delay 0 -clock [get_clocks virtual_io] [get_ports global_en]
set_output_delay 0 -clock [get_clocks virtual_io] [get_ports global_st]
set_false_path -from [get_ports cache_crtl]
set_false_path -from [get_ports global_en]
set_false_path -to [get_ports cache_st]
set_false_path -to [get_ports global_st]

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

# constrain eth clk
create_generated_clock -name RMII_clk_pin -source [get_ports clk] -divide_by 2 [get_ports RMII_rclk];

set_input_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_crsdv]
set_input_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_rxer]
set_input_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_rxd[0]]
set_input_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_rxd[1]]
set_output_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_txen]
set_output_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_txd[0]]
set_output_delay 5 -clock [get_clocks RMII_clk_pin] [get_ports RMII_txd[1]]

# place eth
#set_property -dict { PACKAGE_PIN C9    IOSTANDARD LVCMOS33 } [get_ports { ETH_MDC }]; #IO_L11P_T1_SRCC_16 Sch=eth_mdc
#set_property -dict { PACKAGE_PIN A9    IOSTANDARD LVCMOS33 } [get_ports { ETH_MDIO }]; #IO_L14N_T2_SRCC_16 Sch=eth_mdio
#set_property -dict { PACKAGE_PIN B3    IOSTANDARD LVCMOS33 } [get_ports { ETH_RSTN }]; #IO_L10P_T1_AD15P_35 Sch=eth_rstn
set_property -dict { PACKAGE_PIN D9    IOSTANDARD LVCMOS33 } [get_ports { RMII_crsdv }]; #IO_L6N_T0_VREF_16 Sch=eth_crsdv
set_property -dict { PACKAGE_PIN C10   IOSTANDARD LVCMOS33 } [get_ports { RMII_rxer }]; #IO_L13N_T2_MRCC_16 Sch=eth_rxerr
set_property -dict { PACKAGE_PIN C11   IOSTANDARD LVCMOS33 } [get_ports { RMII_rxd[0] }]; #IO_L13P_T2_MRCC_16 Sch=eth_rxd[0]
set_property -dict { PACKAGE_PIN D10   IOSTANDARD LVCMOS33 } [get_ports { RMII_rxd[1] }]; #IO_L19N_T3_VREF_16 Sch=eth_rxd[1]
set_property -dict { PACKAGE_PIN B9    IOSTANDARD LVCMOS33 } [get_ports { RMII_txen }]; #IO_L11N_T1_SRCC_16 Sch=eth_txen
set_property -dict { PACKAGE_PIN A10   IOSTANDARD LVCMOS33 } [get_ports { RMII_txd[0] }]; #IO_L14P_T2_SRCC_16 Sch=eth_txd[0]
set_property -dict { PACKAGE_PIN A8    IOSTANDARD LVCMOS33 } [get_ports { RMII_txd[1] }]; #IO_L12N_T1_MRCC_16 Sch=eth_txd[1]
set_property -dict { PACKAGE_PIN D5    IOSTANDARD LVCMOS33 } [get_ports { RMII_rclk }]; #IO_L11P_T1_SRCC_35 Sch=eth_refclk
#set_property -dict { PACKAGE_PIN B8    IOSTANDARD LVCMOS33 } [get_ports { ETH_INTN }]; #IO_L12P_T1_MRCC_16 Sch=eth_intn

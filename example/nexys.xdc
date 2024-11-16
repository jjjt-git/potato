# The Potato Processor - A simple processor for FPGAs
# (c) Kristian Klomsten Skordal 2016 <kristian.skordal@wafflemail.net>
# Report bugs and issues on <https://github.com/skordal/potato/issues>
#
# Adapted from arty.xdc for Nexys A7 100T by Jacob Tilger

# Set operating conditions to improve temperature estimation:
set_operating_conditions -airflow 0
set_operating_conditions -heatsink low

# Clock signal:
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {clk}];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {clk}];

# Reset button:
set_property -dict {PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports {reset_n}];

# GPIOs (Buttons):
set_property -dict {PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[0]}];
set_property -dict {PACKAGE_PIN M17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[1]}];
set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[2]}];
set_property -dict {PACKAGE_PIN P17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[3]}];

# GPIO (Switches):
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[4]}];
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[5]}];
set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[6]}];
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[7]}];

# GPIOs (LEDs):
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[8]}];
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[9]}];
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[10]}];
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {gpio_pins[11]}];

# UART0:
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports {uart0_txd}];
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports {uart0_rxd}];

# UART1 (pin 5 and 6 on JA, to match the pins on the PMOD-GPS):
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports {uart1_txd}];
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {uart1_rxd}];

-- The Potato Processor - SoC design for the Arty FPGA board
-- (c) Kristian Klomsten Skordal 2016 <kristian.skordal@wafflemail.net>
-- Report bugs and issues on <https://github.com/skordal/potato/issues>

library ieee;
use ieee.std_logic_1164.all;

entity tb_toplevel is
end entity tb_toplevel;

architecture testbench of tb_toplevel is

	signal clk : std_logic := '0';
	constant clk_period : time := 10 ns;

	signal reset_n : std_logic := '0';

	signal gpio_pins : std_logic_vector(11 downto 0);

	signal uart0_txd : std_logic;
	signal uart0_rxd : std_logic := '1';

	signal uart1_txd : std_logic;
	signal uart1_rxd : std_logic := '1';
	
	signal cache_crtl : std_logic_vector(4 downto 0) := "11111";

begin

	uut: entity work.toplevel
--		generic map (
--			MEMORY_INIT_FILE     => "/home/jacob/Projects/Studium/Beleg/benchmarks/test.mem",
--			DCACHE_MAX_LINE_SIZE => 4,
--			DCACHE_CACHE_DEPTH   => 4
--		)
		port map(
			clk => clk,
			reset_n => reset_n,
			uart0_txd => uart0_txd,
			uart0_rxd => uart0_rxd,
			uart1_txd => uart1_txd,
			uart1_rxd => uart1_rxd,
			
			global_en => '1',
			cache_crtl => cache_crtl,
			trace_enable => "00"
		);

	clock: process
	begin
		clk <= '0';
		wait for clk_period / 2;
		clk <= '1';
		wait for clk_period / 2;
	end process clock;

	stimulus: process
	begin
		reset_n <= '0';
		wait for clk_period * 4;
		reset_n <= '1';

		wait;
	end process stimulus;

end architecture testbench;

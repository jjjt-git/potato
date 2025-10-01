library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pp_dcache_pol_fifo is
	generic (
		WAYS        : integer range 1 to 64 := 4;
		CACHE_DEPTH : integer               := 5
	);
	port (
		clk   : in std_logic;
	
		repl  : out integer range 0 to WAYS - 1;
		index : in  integer range 0 to CACHE_DEPTH - 1;
		
		replace : in std_logic;
		empty   : in boolean
	);
end entity pp_dcache_pol_fifo;

architecture fifo of pp_dcache_pol_fifo is
	type meta_t is array(0 to CACHE_DEPTH - 1) of integer range 0 to WAYS - 1;
	signal meta_a : meta_t;
	
	attribute ram_style           : string;
	attribute ram_style of meta_a : signal is "distributed";
begin
	eval: process(meta_a, index) begin
		repl <= meta_a(index);
	end process eval;
	
	state: process(clk) begin
		if rising_edge(clk) then
			if empty then
				meta_a(index) <= 0;
			elsif replace = '1' then
				-- one case
				-- placement into anywhere => increment counter
				meta_a(index) <= (meta_a(index) + 1) mod WAYS;
			end if;
		end if;
	end process state;
end architecture fifo;
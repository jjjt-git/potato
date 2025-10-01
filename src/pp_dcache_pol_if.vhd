library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.pp_utilities.all;

entity pp_dcache_pol is
	generic (
		WAYS        : integer range 1 to 64 := 4;
		AUXBITS     : integer range 1 to 64 := 1;
		AUXGENERIC  : integer               := 1;
		CACHE_DEPTH : integer               := 5
	);
	port (
		clk   : in std_logic;
	
		repl  : out integer range 0 to WAYS - 1;
		way   : in  integer range 0 to WAYS - 1;
		index : in  integer range 0 to CACHE_DEPTH - 1;
		
		replace : in std_logic;
		update  : in std_logic;
		empty   : in boolean;
		
		aux  : in  std_logic_vector(AUXBITS - 1 downto 0)
	);
end entity pp_dcache_pol;
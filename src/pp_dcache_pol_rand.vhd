library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pp_dcache_pol_rand is
	generic (
		WAYS        : integer range 1 to 64 := 4;
		AUXBITS     : integer range 1 to 64 := 1
	);
	port (
		repl  : out integer range 0 to WAYS - 1;
		
		aux  : in  std_logic_vector(AUXBITS - 1 downto 0)
	);
end entity pp_dcache_pol_rand;

architecture random of pp_dcache_pol_rand is
begin
repl <= to_integer(unsigned(aux)) mod WAYS;
end architecture random;
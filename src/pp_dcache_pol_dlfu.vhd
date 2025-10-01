library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.pp_utilities.all;

entity pp_dcache_pol_dlfu is
	generic (
		WAYS        : integer range 1 to 64 := 4;
		AUXGENERIC  : integer               := 32;
		CACHE_DEPTH : integer               := 5
	);
	port (
		clk   : in std_logic;
	
		repl  : out integer range 0 to WAYS - 1;
		way   : in  integer range 0 to WAYS - 1;
		index : in  integer range 0 to CACHE_DEPTH - 1;
		
		replace : in std_logic;
		update  : in std_logic;
		empty   : in boolean
	);
end entity pp_dcache_pol_dlfu;

architecture dlfu of pp_dcache_pol_dlfu is
	constant line_c_bits : integer := log2(AUXGENERIC);

	subtype index_t is integer range 0 to CACHE_DEPTH - 1;
	subtype way_t   is integer range 0 to WAYS - 1;
	
	subtype line_c  is integer range 0 to AUXGENERIC - 1;
	subtype group_c is integer range 0 to AUXGENERIC * 2 - 1;
	
	type line_c_a is array(way_t) of line_c;
	
	subtype line_c_phys is std_logic_vector(WAYS * line_c_bits - 1 downto 0);
	
	type line_a_t  is array(index_t) of line_c_phys;
	type group_a_t is array(index_t) of group_c;
	
	signal line_a  : line_a_t;
	signal group_a : group_a_t;
	
	signal cur_line  : line_c_a;
	signal cur_group : group_c;
	
	attribute ram_style            : string;
	attribute ram_style of line_a  : signal is "distributed";
	attribute ram_style of group_a : signal is "distributed";
	
	function to_physical(input : line_c_a) return line_c_phys is
		variable work : line_c_phys;
	begin
		for ii in way_t loop
			work((ii + 1) * line_c_bits - 1 downto ii * line_c_bits) := std_logic_vector(to_unsigned(input(ii), line_c_bits));
		end loop;
		return work;
	end function to_physical;
	
	function to_record(input : line_c_phys) return line_c_a is
		variable work : line_c_a;
	begin
		for ii in way_t loop
			work(ii) := to_integer(unsigned(input((ii + 1) * line_c_bits - 1 downto ii * line_c_bits)));
		end loop;
		return work;
	end function to_record;
begin

	cur_line  <= to_record(line_a(index));
	cur_group <= group_a(index);

	eval: process(index, cur_line)
		variable key     : way_t;
		variable min     : line_c;
	begin
		key := 0;
		min := line_c'high;
		
		for ii in way_t loop
			if cur_line(ii) < min then
				min := cur_line(ii);
				key := ii;
			end if;
		end loop;
		
		repl <= key;
	end process eval;
	
	state: process(clk, update, replace, empty, cur_line, cur_group, way)
		variable lwork : line_c_a;
		variable gwork : group_c;
		variable l_we, g_we : std_logic;
	begin
		lwork := cur_line;
		gwork := cur_group;
		l_we := '0';
		g_we := '0';
	
		if update = '1' or replace = '1' then
			-- three cases:
			-- I   empty          => 0 -> counter
			-- II  counter at max => 0 -> counter
			-- III middle         => increment counter
			if empty or cur_group = group_c'high then
				gwork := 0;
				g_we := '1';
			else
				gwork := cur_group + 1;
				g_we := '1';
			end if;
			
			for ii in 0 to way_t'high loop -- line counter loop
				-- five cases:
				-- I   replacement of element          => 0 -> counter
				-- II  access to element without decay => increment counter
				-- III access to element with decay    => (counter >> 1) + 1 -> counter
				-- IV  access to group without decay   => do nothing
				-- V   access to group with decay      => counter >> 1 -> counter
				if way = ii and replace = '1' then -- replace
					lwork(ii) := 0; -- I
					l_we := '1';
				elsif cur_group = group_c'high then -- decay
					if way = ii then -- access to this element
						lwork(ii) := (cur_line(ii) / 2) + 1; -- III
						l_we := '1';
					else -- access to group
						lwork(ii) := cur_line(ii) / 2; -- V
						l_we := '1';
					end if;
				else -- no decay
					if way = ii then -- access to this element
						lwork(ii) := cur_line(ii) + 1; -- II
						l_we := '1';
					end if;
				end if;
			end loop;
		end if;
		
		group_op: if rising_edge(clk) then
			if g_we = '1' then
				group_a(index) <= gwork;
			end if;
		end if;
		
		line_op: if rising_edge(clk) then
			if l_we = '1' then
				line_a(index) <= to_physical(lwork);
			end if;
		end if;
	end process state;
end architecture dlfu;
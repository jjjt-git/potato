library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pp_dcache_pol_dual_lru_mru is
	generic (
		WAYS        : integer range 1 to 64 := 4;
		CACHE_DEPTH : integer               := 5
	);
	port (
		clk   : in std_logic;
	
		repl0 : out integer range 0 to WAYS - 1;
		repl1 : out integer range 0 to WAYS - 1;
		way   : in  integer range 0 to WAYS - 1;
		index : in  integer range 0 to CACHE_DEPTH - 1;
		
		replace : in std_logic;
		update  : in std_logic;
		empty   : in boolean
	);
end entity pp_dcache_pol_dual_lru_mru;

architecture lru_mru of pp_dcache_pol_dual_lru_mru is
	subtype way_t is integer range 0 to WAYS - 1;
	
	type meta_t is array (way_t) of std_logic_vector(way_t);
	
	subtype meta_phys_t is std_logic_vector(WAYS * CACHE_DEPTH - 1 downto 0);
	type meta_a_t is array(0 to CACHE_DEPTH - 1) of meta_phys_t;
	
	signal meta   : meta_t;
	
	signal meta_a : meta_a_t;
	
	attribute ram_style           : string;
	attribute ram_style of meta_a : signal is "distributed";
	
	function to_physical(input : meta_t) return meta_phys_t is
		variable work : meta_phys_t;
	begin
		for ii in way_t loop
			work((ii + 1) * WAYS - 1 downto ii * WAYS) := input(ii);
		end loop;
		return work;
	end function to_physical;
	
	function to_record(input : meta_phys_t) return meta_t is
		variable work : meta_t;
	begin
		for ii in way_t loop
			work(ii) := input((ii + 1) * WAYS - 1 downto ii * WAYS);
		end loop;
		return work;
	end function to_record;
begin
	
	meta <= to_record(meta_a(index));
	
	eval_lru: process(meta)
		variable key : way_t;
	begin
		key := 0;
		
		for ii in 0 to way_t'high loop
			if meta(ii)(way_t'high) = '1' then
				key := ii;
			end if;
		end loop;
		
		repl0 <= key;
	end process eval_lru;
	
	eval_mru: process(meta)
		variable key : way_t;
	begin
		key := 0;
		
		for ii in 0 to way_t'high loop
			if meta(ii)(0) = '1' then
				key := ii;
			end if;
		end loop;
		
		repl1 <= key;
	end process eval_mru;

	state: process(clk, meta, replace, empty, update, way)
		variable repl, this : way_t;
		
		variable meta_u : meta_t;
		variable m_we   : std_logic;
	begin
		meta_u := meta;
		m_we   := '0';
		if update = '1' then
			m_we := '1';
		end if;
		if replace = '1' then
			m_we := '1';
		end if;
		
		for ii in 0 to way_t'high loop -- forall ways
			if empty then
				if way = ii then
					meta_u(ii) := (0 => '1', others => '0');
				else
					meta_u(ii) := (others => '0');
				end if;
			elsif replace = '1' or update = '1' then
				-- two cases
				-- I  placement/access into this                   => record to most recently accessed
				-- II placement/access into less recently accessed => shift

				if way = ii then -- I
					meta_u(ii) := (0 => '1', others => '0');
				else -- II prep for test
					repl := way_t'high;
					this := way_t'high;
					for jj in 0 to way_t'high loop
						if meta(way)(jj) = '1' then -- find age of the replaced
							repl := jj;
						end if;
						
						if meta(ii)(jj) = '1' then -- find age of this
							this := jj;
						end if;
					end loop;
					
					if repl > this then -- test for II
						meta_u(ii) := std_logic_vector(shift_right(unsigned(
							meta(ii)
						), 1));
					end if;
				end if;
			end if;
		end loop;
		
		array_op: if rising_edge(clk) then
			if m_we = '1' then
				meta_a(index) <= to_physical(meta_u);
			end if;
		end if; 
	end process state;
end architecture lru_mru;
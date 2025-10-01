-- THESE ARE FOR REFERENCE
--entity pp_dcache_pol is
--	generic (
--		WAYS        : integer range 1 to 64 := 4;
--		AUXBITS     : integer range 1 to 64 := 1;
--		AUXGENERIC  : integer               := 1;
--		CACHE_DEPTH : integer               := 5
--	);
--	port (
--		clk   : in std_logic;
	
--		repl  : out integer range 0 to WAYS - 1;
--		way   : in  integer range 0 to WAYS - 1;
--		index : in  integer range 0 to CACHE_DEPTH - 1;
		
--		replace : in std_logic;
--		update  : in std_logic;
--		empty   : in boolean;
		
--		aux  : in  std_logic_vector(AUXBITS - 1 downto 0)
--	);
--end entity pp_dcache_pol;
--
--entity pp_dcache_pol_dual is
--	generic (
--		WAYS        : integer range 1 to 64 := 4;
--		AUXBITS     : integer range 1 to 64 := 1;
--		AUXGENERIC  : integer               := 1;
--		CACHE_DEPTH : integer               := 5
--	);
--	port (
--		clk   : in std_logic;
	
--		repl0 : out integer range 0 to WAYS - 1;
--		repl1 : out integer range 0 to WAYS - 1;
--		way   : in  integer range 0 to WAYS - 1;
--		index : in  integer range 0 to CACHE_DEPTH - 1;
		
--		replace : in std_logic;
--		update  : in std_logic;
--		empty   : in boolean;
		
--		aux : in  std_logic_vector(AUXBITS - 1 downto 0)
--	);
--end entity pp_dcache_pol_dual;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.pp_types.all;
use work.pp_utilities.all;
use work.tracing_types.all;

entity pp_dcache_pol_sel is
	generic (
		HAS_LRU    : boolean := true;
		HAS_MRU    : boolean := true;
		HAS_DLFU   : boolean := true;
		HAS_FIFO   : boolean := true;
		HAS_RANDOM : boolean := true;
		
		DLFU_RATE : integer := 32;
		
		ADAPTIVE_HISTORY : integer := 8;
		
		TAG_BITS    : integer  := 8;
		CACHE_DEPTH : integer  := 5;
		WAYNESS     : positive := 4
	);
	port (
		clk : in std_logic;
		rst : in std_logic;
		
		wb_finished : in std_logic;
		wb_tag      : in std_logic_vector(TAG_BITS - 1 downto 0);
		wb_index    : in  integer range 0 to CACHE_DEPTH - 1;
		
		valid : in  std_logic_vector(WAYNESS - 1 downto 0);
		repl  : out integer range 0 to WAYNESS - 1;
		way   : in  integer range 0 to WAYNESS - 1;
		index : in  integer range 0 to CACHE_DEPTH - 1;
		tag   : in  std_logic_vector(TAG_BITS - 1 downto 0);
		tags  : in  std_logic_vector(TAG_BITS * WAYNESS - 1 downto 0);
		
		replace  : in  std_logic;
		update   : in  std_logic;
		finished : out std_logic;
		
		crtl  : in std_logic_vector(4 downto 0); -- bit 0 lru; bit 1 dlfu; bit 2 mru; bit 3 fifo; bit 4 rand
		
		-- trace
		replace_event : out std_logic;
		replace_pol   : out policy_t;
		
		-- Random input
		random : in std_logic_vector(7 downto 0)
	);
end entity pp_dcache_pol_sel;

architecture Behaviour of pp_dcache_pol_sel is
	constant policy_number : integer := 5;
	
	subtype way_t is integer range 0 to WAYNESS - 1;
	subtype index_t is integer range 0 to CACHE_DEPTH - 1;
	subtype tag_t is std_logic_vector(TAG_BITS - 1 downto 0);

	subtype pol_adapt_ptr_t is integer range 0 to ADAPTIVE_HISTORY - 1;
	
	type pol_adapt_idx_r is array(pol_adapt_ptr_t) of index_t;
	
	type pol_adapt_evict_r is array(pol_adapt_ptr_t) of tag_t;
	type pol_adapt_pol_r   is record
		history    : pol_adapt_evict_r;
		refetched  : std_logic_vector(pol_adapt_ptr_t);
	end record;
	type pol_adapt_r is array(0 to policy_number - 1) of pol_adapt_pol_r;
	signal pol_adapt     : pol_adapt_r;
	signal pol_adapt_idx : pol_adapt_idx_r;
	signal nxt           : pol_adapt_ptr_t;
	
	type eviction_note_t is array(0 to policy_number - 1) of tag_t;
	signal pol_eviction_tag   : eviction_note_t;
	signal pol_eviction_valid : std_logic_vector(0 to policy_number - 1);
	
	subtype pol_prio_t is integer range 0 to ADAPTIVE_HISTORY;
	type pol_prio_a is array(0 to policy_number - 1) of pol_prio_t;
	
	signal empty, full : boolean;
	
	signal dlfu_repl, lru_repl, mru_repl, fifo_repl, rand_repl: way_t;
begin
	process(clk) begin -- management for all pol-adaptivity state
		if rising_edge(clk) then
			if rst = '1' then
				nxt <= 0;
			elsif full and replace = '1' then
				nxt <= (nxt + 1) mod (pol_adapt_ptr_t'high + 1);
				pol_adapt_idx(nxt) <= index;
			end if;
		end if;
	
		for pid in pol_eviction_tag'range loop
			if rising_edge(clk) then
				if rst = '1' then
					pol_adapt(pid).refetched <= (others => '1'); -- init with all ones, new entries will start with 0
				elsif full then -- if space available ignore
					if wb_finished = '1' then -- need to update active history
						for ii in pol_adapt_ptr_t loop
							-- three cases:
							-- I   write-out/read-in of line in history     -> mark as refetched
							-- II  write-out/read-in of line not in history -> do nothing
							-- III access to line in history                -> mark as refetched (would have caused refetch if followed)
							if wb_tag = pol_adapt(pid).history(ii) and wb_index = pol_adapt_idx(ii) then -- I and III
								pol_adapt(pid).refetched(ii) <= '1';
							end if; -- else would be II
						end loop;
					end if;
					
					if replace = '1' then -- push new history
						pol_adapt(pid).history(nxt) <= pol_eviction_tag(pid);
						pol_adapt(pid).refetched(nxt) <= '0';
					end if;
				elsif update = '1' then
					for ii in pol_adapt_ptr_t loop
						if tag = pol_adapt(pid).history(ii) and index = pol_adapt_idx(ii) then
							pol_adapt(pid).refetched(ii) <= '1';
						end if;
					end loop;
				end if;
			end if;
		end loop;
	end process;
	
	process(tags, valid, lru_repl, dlfu_repl, mru_repl, fifo_repl, rand_repl)
		type tag_a_t is array(way_t) of tag_t;
		variable tag_a : tag_a_t;
	begin
		for ii in tag_a_t'range loop
			tag_a(ii) := tags((ii + 1) * tag_t'length - 1 downto ii * tag_t'length);
		end loop;
	
		pol_eviction_tag(0) <= tag_a(lru_repl);
		pol_eviction_tag(1) <= tag_a(dlfu_repl);
		pol_eviction_tag(2) <= tag_a(mru_repl);
		pol_eviction_tag(3) <= tag_a(fifo_repl);
		pol_eviction_tag(4) <= tag_a(rand_repl);
	
		pol_eviction_valid(0) <= valid(lru_repl);
		pol_eviction_valid(1) <= valid(dlfu_repl);
		pol_eviction_valid(2) <= valid(mru_repl);
		pol_eviction_valid(3) <= valid(fifo_repl);
		pol_eviction_valid(4) <= valid(rand_repl);
	end process;
	
	policy_sel: process(crtl, dlfu_repl, lru_repl, mru_repl, fifo_repl, rand_repl, pol_adapt, valid, index, replace)
		variable prios : pol_prio_a;
		variable en : std_logic_vector(prios'range);
		
		variable empty_slot    : boolean;
		variable empty_slot_nr : way_t;
		variable pol_empty     : boolean;
		
		variable ready : std_logic;
		variable way   : way_t;
	begin
		if HAS_LRU then
			en(0) := crtl(0);
		else
			en(0) := '0';
		end if;
		
		if HAS_DLFU then
			en(1) := crtl(1);
		else
			en(1) := '0';
		end if;
		
		if HAS_MRU then
			en(2) := crtl(2);
		else
			en(2) := '0';
		end if;
		
		if HAS_FIFO then
			en(3) := crtl(3);
		else
			en(3) := '0';
		end if;
		
		if HAS_RANDOM then
			en(4) := crtl(4);
		else
			en(4) := '0';
		end if;
		
		empty_slot_nr := 0;
		empty_slot := false;
		pol_empty  := true;
		for ii in 0 to way_t'high loop
			if valid(ii) = '0' then
				empty_slot := true;
				empty_slot_nr := ii;
			end if;
			if valid(ii) = '1' then
				pol_empty := false;
			end if;
		end loop;
		
		empty <= pol_empty;
		full  <= not empty_slot;
		
		for pid in prios'range loop
			if en(pid) = '1' then
				prios(pid) := 0;
				for ii in pol_adapt_ptr_t loop
					if pol_adapt(pid).refetched(ii) = '1' then
						prios(pid) := prios(pid) + 1;
					end if;
				end loop;
			else
				prios(pid) := pol_prio_t'high;
			end if;
		end loop;
		
		replace_pol.random <= prios(4);
		replace_pol.fifo   <= prios(3);
		replace_pol.lru    <= prios(0);
		replace_pol.mru    <= prios(2);
		replace_pol.dlfu   <= prios(1);
		
		-- general priority with ties
		-- random > fifo > dlfu > lru > mru
		-- bit 0 lru; bit 1 dlfu; bit 2 mru; bit 3 fifo; bit 4 rand
		if
				en(2) = '1' and
				(prios(2) < prios(0) or en(0) = '0') and
				(prios(2) < prios(1) or en(1) = '0') and
				(prios(2) < prios(3) or en(3) = '0') and
				(prios(2) < prios(4) or en(4) = '0') then
			-- mru is best in recent history
			ready := '1';
			way   := mru_repl;
			
			replace_event <= replace;
			replace_pol.active <= POL_MRU;
		elsif
				en(0) = '1' and
				(prios(0) < prios(1) or en(1) = '0') and
				(prios(0) < prios(3) or en(3) = '0') and
				(prios(0) < prios(4) or en(4) = '0') then
			-- lru is best in recent history
			ready := '1';
			way   := lru_repl;
			
			replace_event <= replace;
			replace_pol.active <= POL_LRU;
		elsif
				en(1) = '1' and
				(prios(1) < prios(3) or en(3) = '0') and
				(prios(1) < prios(4) or en(4) = '0') then
			-- dlfu is best in recent history
			ready := '1';
			way   := dlfu_repl;
			
			replace_event <= replace;
			replace_pol.active <= POL_DLFU;
		elsif
				en(3) = '1' and
				(prios(3) < prios(4) or en(4) = '0') then
			-- fifo is best in recent history
			ready := '1';
			way   := fifo_repl;
			
			replace_event <= replace;
			replace_pol.active <= POL_FIFO;
		elsif
				en(4) = '1' then
			-- rand is best in recent history
			ready := '1';
			way   := rand_repl;
			
			replace_event <= replace;
			replace_pol.active <= POL_RANDOM;
		else
			ready := '1';
			way   := 0;
			
			replace_event <= '0';
			replace_pol.active <= POL_NONE;
		end if;
		
		if empty_slot then
			repl     <= empty_slot_nr;
			finished <= '1';
		else
			repl     <= way;
			finished <= ready;	
		end if;
	end process policy_sel;
	
	pol_dlfu: entity work.pp_dcache_pol_dlfu
		generic map (
			WAYS        => WAYNESS,
			CACHE_DEPTH => CACHE_DEPTH,
			AUXGENERIC  => DLFU_RATE
		) port map (
			clk => clk,
		
			repl  => dlfu_repl,
			way   => way,
			index => index,
			
			replace => replace,
			update  => update,
			empty   => empty
		);
	
	pol_fifo: entity work.pp_dcache_pol_fifo
		generic map (
			WAYS        => WAYNESS,
			CACHE_DEPTH => CACHE_DEPTH
		) port map (
			clk => clk,
		
			repl  => fifo_repl,
			index => index,
			
			replace => replace,
			empty   => empty
		);
	
	pol_rand: entity work.pp_dcache_pol_rand
		generic map (
			WAYS        => WAYNESS,
			AUXBITS     => 8
		) port map (
			repl  => rand_repl,
			aux => random
		);
	
	pol_lru_mru: entity work.pp_dcache_pol_dual_lru_mru
		generic map (
			WAYS        => WAYNESS,
			CACHE_DEPTH => CACHE_DEPTH
		) port map (
			clk => clk,
		
			repl0 => lru_repl,
			repl1 => mru_repl,
			way   => way,
			index => index,
			
			replace => replace,
			update  => update,
			empty   => empty
		);
end architecture Behaviour;
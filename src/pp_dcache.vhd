-- The Potato Processor - A simple processor for FPGAs
-- (c) Kristian Klomsten Skordal 2014 - 2015 <kristian.skordal@wafflemail.net>
-- Report bugs and issues on <https://github.com/skordal/potato/issues>

-- Adaptive cache implementation
-- (c) Jacob Tilger 2024 - 2025 <jacob.tilger@mailbox.tu-dresden.de>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.pp_types.all;
use work.pp_utilities.all;

--! @brief DCache implementation
entity pp_dcache is
	generic(
		REGION_BASE      : std_logic_vector(31 downto 0) := x"00000000"; --! The base address of the cached region.
		REGION_LD_LEN    : integer                       := 20;          --! The binary logarithm of the size of the cached region, i.e. the length of the address-offset.
		MAX_LINE_SIZE    : integer                       := 8;           --! Maximum number of words per data cache line.
		WAYNESS          : integer                       := 4;           --! Number of ways of the cache array, i.e. physical assoziativity
		CACHE_DEPTH      : integer                       := 128;         --! Number of cache line sets in the data cache.

		HAS_DECAYING_LFU : boolean                       := true;        --! True, if DLFU should be available.
		DLFU_RATE        : integer                       := 32;          --! Number of accesses to a group between decays.
		
		HAS_LRU          : boolean                       := true;
		
		ADAPTIVE_HISTORY : integer                       := 8
	);
	port(
		clk   : in std_logic;
		reset : in std_logic;

		-- Processor data memory signals:
		mem_address   : in  std_logic_vector(31 downto 0);
		mem_data_in   : in  std_logic_vector(31 downto 0); -- Data in from the bus
		mem_data_out  : out std_logic_vector(31 downto 0); -- Data out to the bus
		mem_data_size : in  std_logic_vector( 1 downto 0);
		mem_read_req  : in  std_logic;
		mem_read_ack  : out std_logic;
		mem_write_req : in  std_logic;
		mem_write_ack : out std_logic;
		
		inval : in std_logic;
		global_enable : in std_logic;
		crtl  : in std_logic_vector(1 downto 0); -- bit 0 lru; bit 1 dlfu

		-- Wishbone interface:
		wb_inputs  : in wishbone_master_inputs;
		wb_outputs : out wishbone_master_outputs
	);
end entity pp_dcache;

architecture behaviour of pp_dcache is

	-- address splitting constants
	constant boffset_bits : integer := 2;
	constant region_bits  : integer := 32 - REGION_LD_LEN;
	constant woffset_bits : integer := log2(MAX_LINE_SIZE);
	constant index_bits   : integer := log2(CACHE_DEPTH) - log2(WAYNESS);
	constant tag_bits     : integer := 32 - region_bits - boffset_bits - woffset_bits - index_bits;

	-- address part types
	subtype addr_region_t  is std_logic_vector(region_bits  - 1 downto 0);
	subtype addr_boffset_t is std_logic_vector(boffset_bits - 1 downto 0);
	subtype addr_index_t   is std_logic_vector(index_bits   - 1 downto 0);
	subtype addr_tag_t     is std_logic_vector(tag_bits     - 1 downto 0);
	subtype addr_woffset_t is std_logic_vector(woffset_bits - 1 downto 0);

	subtype addr_line_t is std_logic_vector(31 downto woffset_bits + boffset_bits);

	-- selection types
	subtype index_t  is integer range 0 to CACHE_DEPTH - 1;
	subtype way_t    is integer range 0 to WAYNESS - 1;
	subtype offset_t is integer range 0 to MAX_LINE_SIZE - 1;

	subtype word_mask_t is std_logic_vector(3 downto 0);
	type word_mask_a    is array(0 to MAX_LINE_SIZE - 1) of word_mask_t;
	subtype line_mask_t is std_logic_vector(MAX_LINE_SIZE * 4 - 1 downto 0);
	
	-- cache line types
	subtype cache_line_t    is std_logic_vector(MAX_LINE_SIZE * 32 - 1 downto 0);
	type cache_line_words_a is array(0 to MAX_LINE_SIZE - 1) of std_logic_vector(31 downto 0);
	type cache_line_a       is array(0 to CACHE_DEPTH - 1) of cache_line_t;

	-- cache tag type
	subtype cache_tag_t is addr_tag_t;
	type cache_tag_a    is array(0 to index_t'high) of cache_tag_t;

	-- cache metadata type
	subtype dlfu_group_c is integer range 0 to DLFU_RATE - 1;
	subtype dlfu_line_c  is integer range 0 to DLFU_RATE * 2 - 1;
	type dlfu_meta_t is record
		group_c : dlfu_group_c;
		line_c  : dlfu_line_c;
	end record;
	
	subtype lru_meta_t is way_t;
	
	type cache_meta_t is record
		dirty : std_logic;
	end record;
	
	type cache_dlfu_meta_a is array(0 to index_t'high) of dlfu_meta_t;
	type cache_lru_meta_a  is array(0 to index_t'high) of lru_meta_t;
	type cache_meta_a      is array(0 to index_t'high) of cache_meta_t;
	
	-- policy adaptivity
	subtype pol_adapt_ptr_t is integer range 0 to ADAPTIVE_HISTORY - 1;
	
	type pol_adapt_evict_r is array(0 to pol_adapt_ptr_t'high) of cache_tag_t;
	type pol_adapt_pol_r is record
		history    : pol_adapt_evict_r;
		refetched  : std_logic_vector(0 to pol_adapt_ptr_t'high);
		nxt        : pol_adapt_ptr_t;
	end record;
	type pol_adapt_r is array(0 to 1) of pol_adapt_pol_r;
	signal pol_adapt : pol_adapt_r;
	
	type eviction_note_t is array(0 to 1) of cache_tag_t;
	signal pol_eviction_tag   : eviction_note_t;
	signal pol_eviction_valid : std_logic_vector(0 to 1);
	
	subtype pol_prio_t is integer range 0 to ADAPTIVE_HISTORY;
	type pol_prio_a is array(0 to 1) of pol_prio_t;

	-- cache memories
	signal data_a  : cache_line_a;
	signal tag_a   : cache_tag_a;
	signal valid_a : std_logic_vector(index_t'high downto 0);
	
	signal meta_a      : cache_meta_a;
	signal meta_a_lru  : cache_lru_meta_a;
	signal meta_a_dlfu : cache_dlfu_meta_a;

	attribute ram_style            : string;
	attribute ram_style of data_a  : signal is "block";

	-- signals for response control
	signal cache_hit  : std_logic;
	signal hit_way    : way_t;
	signal hit_line_shifted : cache_line_t;
	signal hit_line_a : cache_line_words_a;
	signal pt_was_we  : std_logic;
	signal ack_read_req : std_logic;

	signal response, response_latch : std_logic_vector(31 downto 0);

	-- address split signals
	signal addr_latched   : std_logic_vector(31 downto 0);
	signal addr_region    : addr_region_t;
	signal addr_woffset   : addr_woffset_t;
	signal addr_boffset   : addr_boffset_t;
	signal addr_index     : addr_index_t;
	signal addr_tag       : addr_tag_t;
	signal addr_region_l  : addr_region_t;
	signal addr_woffset_l : addr_woffset_t;
	signal addr_boffset_l : addr_boffset_t;
	signal addr_index_l   : addr_index_t;
	signal addr_tag_l     : addr_tag_t;

	-- signals for replace control
	signal repl_way   : way_t;

	-- controller signals
	type main_state_t is (IDLE,
		PASS_THROUGH, PASS_THROUGH_RESPOND,
		WRITE_RESPOND,
		READ_RESPOND, REFILL, REPLACE, WRITE_BACK);
	signal main_state : main_state_t;

	signal in_segment, need_wb, crtl_aux : std_logic;
	signal access_mask, access_mask_shifted : line_mask_t;
	signal data, data_shifted : cache_line_t;

	signal pol_finished : std_logic;
	signal wb_start, wb_stop : offset_t;
	signal wb_rbuffer_words, wb_wbuffer_words : cache_line_words_a;
	signal wb_rbuffer_line, wb_wbuffer_line : cache_line_t;

	type wb_mode_t is (IDLE, WRITE, READ, WRITE_WAIT, READ_WAIT);
	signal wb_mode : wb_mode_t;
	signal wb_tag  : addr_tag_t;
	signal wb_mask : line_mask_t;
	signal wb_mask_a : word_mask_a;
	signal wb_end_tick : std_logic;

	signal pol_update, normal_cycle, pol_replace, resp_rdata : std_logic;
	signal pol_index : addr_index_t;
	signal pol_way   : way_t;

	-- dlfu policy signals
	signal dlfu_repl, lru_repl : way_t;
	
	-- block ram operator signals
	signal rdata, wdata : cache_line_t;
	signal wmask  : line_mask_t;
	signal aindex : index_t;

	-- helper signal
	signal index_h, index_r : index_t;
	signal soft_reset : std_logic;

	-- helper functions
	function get_index(
		entry : addr_index_t;
		way   : way_t
	) return index_t is begin
		return to_integer(unsigned(entry)) * WAYNESS + way;
	end function;

	function get_address(
		region : addr_region_t;
		entry  : addr_index_t;
		tag    : addr_tag_t
	) return addr_line_t is begin
		return region & tag & entry;
	end function;

	function get_mask(
		size : std_logic_vector(1 downto 0)
	) return word_mask_t is begin
		case size is
			when b"01"  => return b"0001";
			when b"10"  => return b"0011";
			when others => return b"1111";
		end case;
	end function;

	function get_shift(
		word : addr_woffset_t;
		byte : addr_boffset_t
	) return integer is begin
		return to_integer(unsigned(word)) * 4 + to_integer(unsigned(byte));
	end function;

begin
	assert is_pow2(MAX_LINE_SIZE) report "Cache line size must be a power of 2!" severity FAILURE;
	assert is_pow2(CACHE_DEPTH) report "Cache depth must be a power of 2!" severity FAILURE;
	assert is_pow2(WAYNESS) report "Wayness must be a power of 2!" severity FAILURE;

	-- splitting of address
	-----------------------------------
	-- region | tag | index | offset --
	-----------------------------------
	addr_region  <= mem_address(31 downto 32 - region_bits);
	addr_tag     <= mem_address(31 - region_bits downto 32 - region_bits - tag_bits);
	addr_index   <= mem_address(31 - region_bits - tag_bits downto 32 - region_bits - tag_bits - index_bits);
	addr_woffset <= mem_address(31 - region_bits - tag_bits - index_bits downto 32 - region_bits - tag_bits - index_bits - woffset_bits);
	addr_boffset <= mem_address(31 - region_bits - tag_bits - index_bits - woffset_bits downto 0);
	
	addr_region_l  <= addr_latched(31 downto 32 - region_bits);
	addr_tag_l     <= addr_latched(31 - region_bits downto 32 - region_bits - tag_bits);
	addr_index_l   <= addr_latched(31 - region_bits - tag_bits downto 32 - region_bits - tag_bits - index_bits);
	addr_woffset_l <= addr_latched(31 - region_bits - tag_bits - index_bits downto 32 - region_bits - tag_bits - index_bits - woffset_bits);
	addr_boffset_l <= addr_latched(31 - region_bits - tag_bits - index_bits - woffset_bits downto 0);
	
	-- enforce alignment
	enforce_algn: process(clk) begin
		if rising_edge(clk) then
			if mem_read_req = '1' or mem_write_req = '1' then
				if mem_data_size = b"10" then
					assert addr_boffset(0) = '0' report "Access must be half word aligned!" severity FAILURE;
				end if;
				
				if mem_data_size = b"11" or mem_data_size = b"00" then
					assert addr_boffset = b"00" report "Access must be word aligned!" severity FAILURE;
				end if;
			end if;
		end if;
	end process enforce_algn;

	latch_response: process (clk) begin
		if rising_edge(clK) then
			if reset = '1' then
				response_latch <= (others => '0');
			elsif ack_read_req = '1' then
				response_latch <= response;
			end if;
		end if;
	end process latch_response;

	response <= std_logic_vector(shift_right(unsigned(hit_line_a(to_integer(unsigned(addr_woffset_l)))), 8 * to_integer(unsigned(addr_boffset_l))));
	mem_data_out <= response when ack_read_req = '1' else response_latch;
	mem_read_ack <= ack_read_req;

	in_segment <=
		'0' when global_enable = '0' else
		'1' when addr_region = REGION_BASE(31 downto 32 - region_bits) else
		'0';

	access_mask <= std_logic_vector(resize(unsigned(get_mask(mem_data_size)), 4 * MAX_LINE_SIZE));
	access_mask_shifted <= std_logic_vector(shift_left(unsigned(access_mask), get_shift(addr_woffset, addr_boffset)));
	data <= std_logic_vector(resize(unsigned(mem_data_in), 32 * MAX_LINE_SIZE));
	data_shifted <= std_logic_vector(shift_left(unsigned(data), get_shift(addr_woffset, addr_boffset) * 8));
	hit_line_shifted <= rdata when resp_rdata = '1' else wb_rbuffer_line;

	decompose_lines: for ii in 0 to MAX_LINE_SIZE - 1 generate
		hit_line_a(ii) <= hit_line_shifted(32 * ii + 31 downto 32 * ii);

		wb_wbuffer_words(ii) <= wb_wbuffer_line(32 * (ii + 1) - 1 downto 32 * ii);
		wb_rbuffer_line(32 * (ii + 1) - 1 downto 32 * ii) <= wb_rbuffer_words(ii);
		wb_mask_a(ii) <= wb_mask(4 * (ii + 1) - 1 downto 4 * ii);
	end generate decompose_lines;

	tag_lookup: process(addr_tag, addr_index, tag_a, valid_a)
		variable hit : std_logic;
		variable way : way_t;
	begin
		hit := '0';
		way := 0;
		for ii in 0 to WAYNESS - 1 loop
			if valid_a(get_index(addr_index, ii)) = '1' and tag_a(get_index(addr_index, ii)) = addr_tag then
				hit := '1';
				way := ii;
			end if;
		end loop;
		hit_way <= way;
		cache_hit <= hit;
	end process tag_lookup;

	wb_outputs.sel <= wb_mask_a(wb_start);

	wb_outputs.adr <= get_address(addr_region_l, addr_index_l, wb_tag) & std_logic_vector(to_unsigned(wb_start, addr_woffset'length)) & b"00";
	wb_outputs.dat <= wb_wbuffer_words(wb_start);

	wb_signals: process(wb_mode) begin
		if wb_mode = WRITE or wb_mode = WRITE_WAIT
		then wb_outputs.we <= '1';
		else wb_outputs.we <= '0';
		end if;

		if wb_mode = IDLE
		then wb_outputs.cyc <= '0';
		else wb_outputs.cyc <= '1';
		end if;

		if wb_mode = WRITE or wb_mode = READ
		then wb_outputs.stb <= '1';
		else wb_outputs.stb <= '0';
		end if;
	end process wb_signals;

	aindex <= index_r when main_state = REFILL else index_h;
	
	operator: process(clk) begin
		if rising_edge(clk) then
			rdata <= data_a(aindex);
			for ii in 0 to MAX_LINE_SIZE * 4 - 1 loop
				if wmask(ii) = '1' then
					data_a(aindex)(8 * (ii + 1) - 1 downto 8 * ii) <= wdata(8 * (ii + 1) - 1 downto 8 * ii);
				end if;
			end loop;
		end if;
	end process operator;

	index_r <= get_index(addr_index, repl_way);
	index_h <= get_index(addr_index, hit_way);
	
	wb_internal_signaling: process(wb_mode, wb_inputs.ack, wb_start, wb_stop) begin
		case wb_mode is
			when IDLE       => wb_end_tick <= '0';
			when READ_WAIT  => wb_end_tick <= '0';
			when WRITE_WAIT => wb_end_tick <= '0';
			when others =>
				if wb_inputs.ack = '1' and wb_start = wb_stop then
					wb_end_tick <= '1';
				else
					wb_end_tick <= '0';
				end if;
		end case;
	end process wb_internal_signaling;

	controller: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				main_state <= IDLE;
				wb_mode <= IDLE;
				wmask <= (others => '0');
				valid_a <= (others => '0');
				normal_cycle <= '1';
				resp_rdata   <= '0';
				soft_reset <= '0';
			else
				case wb_mode is
					when IDLE       => wb_mode <= IDLE;
					when READ_WAIT  => wb_mode <= READ;
					when WRITE_WAIT => wb_mode <= WRITE;
					when others =>
						if wb_inputs.ack = '1' then
							if wb_start = wb_stop then
								wb_mode <= IDLE;
							else
								if wb_mode = WRITE then
									wb_mode <= WRITE_WAIT;
								else
									wb_mode <= READ_WAIT;
								end if;
								wb_start <= wb_start + 1;
							end if;

							if wb_mode = READ then
								wb_rbuffer_words(wb_start) <= wb_inputs.dat;
							end if;
						end if;
				end case;
					
				if inval = '1' or soft_reset = '1' then
					if wb_mode = IDLE then
						soft_reset <= '0';
					else
						soft_reset <= '1';
					end if;
					main_state <= IDLE;
					valid_a <= (others => '0');
					normal_cycle <= '1';
					resp_rdata   <= '0';
				else
					case main_state is
						when IDLE =>
							normal_cycle  <= '1';
							resp_rdata    <= '0';
							ack_read_req  <= '0';
							mem_write_ack <= '0';
							if mem_read_req = '1' or mem_write_req = '1' then -- pseudo-transition to addr-decode
								addr_latched <= mem_address;
								if in_segment = '0' then -- pass-through
									wb_start <= to_integer(unsigned(addr_woffset));
									wb_stop  <= to_integer(unsigned(addr_woffset));
									wb_tag   <= addr_tag;
									wb_mask  <= access_mask_shifted;
	
									wb_wbuffer_line <= data_shifted;
	
									if mem_read_req = '1' then
										wb_mode <= READ;
									else
										wb_mode <= WRITE;
									end if;
	
									main_state <= PASS_THROUGH;
								elsif mem_read_req = '1' then
									need_wb <= '0';
									if cache_hit = '1' then
										main_state <= READ_RESPOND;
									else
										wb_tag <= addr_tag;
	
										wb_start <= 0;
										wb_stop  <= MAX_LINE_SIZE - 1;
										wb_mask  <= (others => '1');
										wb_mode  <= READ;
	
										main_state <= REFILL;
									end if;
								else
									if cache_hit = '1' then
										wdata <= data_shifted;
										wmask <= access_mask_shifted;
										
										main_state <= WRITE_RESPOND;
									else -- pass-through
										wb_start <= to_integer(unsigned(addr_woffset));
										wb_stop  <= to_integer(unsigned(addr_woffset));
										wb_tag   <= addr_tag;
										wb_mask  <= access_mask_shifted;
					
										wb_wbuffer_line <= data_shifted;
					
										wb_mode <= WRITE;
	
										main_state <= PASS_THROUGH;
									end if;
								end if;
							end if;
	
						when READ_RESPOND =>
							if need_wb = '1' then
								main_state <= WRITE_BACK;
							else
								ack_read_req <= '1';
								resp_rdata   <= '1';
								main_state <= IDLE;
							end if;
						when WRITE_BACK =>
							if wb_mode = IDLE then
								ack_read_req <= '1';
								main_state <= IDLE;
							end if;
						when REFILL =>
							normal_cycle <= '0';
							if wb_mode = IDLE and pol_finished = '1' then
								if valid_a(index_r) = '1' and meta_a(index_r).dirty = '1' then
									wb_tag <= tag_a(index_r);
									crtl_aux <= '1';
			   
									wb_start <= 0;
									wb_stop  <= MAX_LINE_SIZE - 1;
									wb_mask  <= (others => '1');
			
									need_wb <= '1';
								else
									crtl_aux <= '0';
								end if;
							
								tag_a  (index_r) <= addr_tag_l;
								meta_a(index_r).dirty <= '0';
								valid_a(index_r) <= '1';
								wdata <= wb_rbuffer_line;
								wmask <= (others => '1');
								
								main_state <= REPLACE;
							end if;
						when REPLACE =>
							wmask <= (others => '0');
							if crtl_aux = '1' then
								wb_wbuffer_line <= rdata;
								wb_mode <= WRITE;
							end if;
	
							main_state <= READ_RESPOND;
	
						when WRITE_RESPOND =>
							wmask <= (others => '0');
							meta_a(index_h).dirty <= '1';
							mem_write_ack <= '1';
	
							main_state <= IDLE;
	
						when PASS_THROUGH =>
							if wb_mode = IDLE then
								pt_was_we <= mem_write_req;
								main_state <= PASS_THROUGH_RESPOND;
							end if;
						when PASS_THROUGH_RESPOND =>
							ack_read_req  <= not pt_was_we;
							mem_write_ack <= pt_was_we;
							main_state <= IDLE;
					end case;
				end if;
			end if;
		end if;
	end process controller;

	pol_update  <= normal_cycle when main_state = READ_RESPOND or main_state = WRITE_RESPOND else '0';
	pol_replace <= '1' when main_state = REPLACE else '0';
	pol_index   <= addr_index_l;
	pol_way     <= repl_way when main_state = REPLACE else hit_way;
	
	policy_sel: process(crtl, dlfu_repl, lru_repl, pol_adapt)
		variable prios : pol_prio_a;
		variable en : std_logic_vector(prios'range);
	begin
		if HAS_LRU then
			en(0) := crtl(0);
		else
			en(0) := '0';
		end if;
		
		if HAS_DECAYING_LFU then
			en(1) := crtl(1);
		else
			en(1) := '0';
		end if;
		
		for pid in prios'range loop
			if en(0) = '1' then
				prios(pid) := 0;
				for ii in 0 to pol_adapt_ptr_t'high loop
					if pol_adapt(pid).refetched(ii) = '1' then
						prios(pid) := prios(pid) + 1;
					end if;
				end loop;
			else
				prios(pid) := pol_prio_t'high;
			end if;
		end loop;
		
		case en is
			when b"11"  => -- both enabled
				if prios(1) < prios(0) then -- dlfu has less misses in history
					repl_way <= dlfu_repl;
					pol_finished <= '1';
				else
					repl_way <= lru_repl;
					pol_finished <= '1';
				end if;
			when b"10"  => -- dlfu enabled
				repl_way <= dlfu_repl;
				pol_finished <= '1';
			when b"01"  => -- lru  enabled
				repl_way <= lru_repl;
				pol_finished <= '1';
			when others => -- none enabled
				repl_way <= 0;
				pol_finished <= '1';
		end case;
	end process policy_sel;
	
	pol_eviction_tag(0)   <= tag_a  (get_index(pol_index, lru_repl));
	pol_eviction_valid(0) <= valid_a(get_index(pol_index, lru_repl));
	pol_eviction_tag(1)   <= tag_a  (get_index(pol_index, dlfu_repl));
	pol_eviction_valid(1) <= valid_a(get_index(pol_index, dlfu_repl));
	
	policy_sel_st: if HAS_DECAYING_LFU and HAS_LRU generate
		process(clk) -- management for all pol-adaptivity state
			variable nxt : pol_adapt_ptr_t;
		begin
			for pid in pol_eviction_tag'range loop
				nxt := pol_adapt(pid).nxt;
				if rising_edge(clk) then
					if reset = '1' then
						pol_adapt(pid).refetched <= (others => '1'); -- init with all ones, new entries will start with 0
						pol_adapt(pid).nxt <= 0;
					elsif pol_eviction_valid(pid) = '1' then -- if line was not valid ignore
						if wb_end_tick = '1' then -- need to update active history
							for ii in 0 to pol_adapt_ptr_t'high loop
								-- three cases:
								-- I   write-out/read-in of line in history -> mark as refetched
								-- II  write-out/read-in of line not in history -> do nothing
								if wb_tag = pol_adapt(pid).history(ii) then -- I
									pol_adapt(pid).refetched(ii) <= '1';
								end if; -- else would be II
							end loop;
						end if;
						
						if pol_replace = '1' then -- push new history
							pol_adapt(pid).history(nxt) <= pol_eviction_tag(pid);
							pol_adapt(pid).refetched(nxt) <= '0';
							pol_adapt(pid).nxt <= (nxt + 1) mod (pol_adapt_ptr_t'high + 1);
						end if;
					end if;
				end if;
			end loop;
		end process;
	end generate policy_sel_st;
	
	policy_dlfu: if HAS_DECAYING_LFU generate
		eval: process(pol_index, meta_a_dlfu, valid_a)
			variable index : index_t;
			variable key   : way_t;
			variable min   : dlfu_line_c;
		begin
			key := 0;
			min := dlfu_line_c'high;
			
			for ii in 0 to way_t'high loop
				index := get_index(pol_index, ii);
				
				if valid_a(index) = '0' then
					min := 0;
					key := ii;
				elsif meta_a_dlfu(index).line_c < min then
					min := meta_a_dlfu(index).line_c;
					key := ii;
				end if;
			end loop;
			
			dlfu_repl <= key;
		end process eval;
		
		state: process(clk, valid_a, pol_index)
			variable index : index_t;
			
			variable empty : boolean;
		begin
			empty := true;
			for ii in 0 to way_t'high loop
				if valid_a(get_index(pol_index, ii)) = '1' then
					empty := false;
				end if;
			end loop;
			
			if rising_edge(clk) then
				if pol_update = '1' or pol_replace = '1' then
					for ii in 0 to way_t'high loop -- group counter mngt
						index := get_index(pol_index, ii);
						-- three cases:
						-- I   empty          => 0 -> counter
						-- II  counter at max => 0 -> counter
						-- III middle         => increment counter
						if empty or meta_a_dlfu(index).group_c = dlfu_group_c'high then
							meta_a_dlfu(index).group_c <= 0;
						else
							meta_a_dlfu(index).group_c <= meta_a_dlfu(index).group_c + 1;
						end if;
					end loop;
					
					for ii in 0 to way_t'high loop -- line counter loop
						index := get_index(pol_index, ii);
						-- five cases:
						-- I   replacement of element          => 0 -> counter
						-- II  access to element without decay => increment counter
						-- III access to element with decay    => (counter >> 1) + 1 -> counter
						-- IV  access to group without decay   => do nothing
						-- V   access to group with decay      => counter >> 1 -> counter
						if pol_way = ii and pol_replace = '1' then -- replace
							meta_a_dlfu(index).line_c <= 0; -- I
						elsif meta_a_dlfu(index).group_c = dlfu_group_c'high then -- decay
							if pol_way = ii then -- access to this element
								meta_a_dlfu(index).line_c <= (meta_a_dlfu(index).line_c / 2) + 1; -- III
							else -- access to group
								meta_a_dlfu(index).line_c <= meta_a_dlfu(index).line_c / 2; -- V
							end if;
						else -- no decay
							if pol_way = ii then -- access to this element
								meta_a_dlfu(index).line_c <= meta_a_dlfu(index).line_c + 1; -- II
							end if;
						end if;
					end loop;
				end if;
			end if;
		end process state;
	end generate policy_dlfu;
	
	policy_lru: if HAS_LRU generate
		eval: process(pol_index, meta_a_lru, valid_a)
			variable index : index_t;
			variable key   : way_t;
			variable hit   : boolean;
		begin
			key := 0;
			hit := false;
			
			for ii in 0 to way_t'high loop
				index := get_index(pol_index, ii);
				
				if valid_a(index) = '0' then
					key := ii;
					hit := true;
				elsif not hit and meta_a_lru(index) = ii then
					key := ii;
				end if;
			end loop;
			
			lru_repl <= key;
		end process eval;
		
		state: process(clk, pol_index, valid_a, pol_way)
			variable index, index_next : index_t;
			
			variable nxt, head : way_t;
			variable found : bit_vector(way_t'high downto 0);
			variable empty : boolean;
		begin
			found := (others => '0');
			empty := true;
			head  := 0;
			nxt   := 0;
			
			for ii in 0 to way_t'high loop -- build lut
				index := get_index(pol_index, ii);
				
				if valid_a(index) = '1' then
					found(meta_a_lru(index)) := '1';
					empty := false;
				end if;
			end loop;
			
			for ii in 0 to way_t'high loop -- get ptrs
				index := get_index(pol_index, ii);
				
				if found(ii) = '0' then -- head is not referenced by any other element
					head := ii;
				end if;
				
				if meta_a_lru(index) = pol_way then -- next element is the one referencing target
					nxt := ii;
				end if;
			end loop;
			
			index      := get_index(pol_index, pol_way);
			index_next := get_index(pol_index, nxt);
		
			if rising_edge(clk) then
				-- four cases:
				-- I   placement into invalid, set empty     => cur ref self
				-- II  placement into invalid, set not empty => cur ref head
				-- III placement/update into tail            => cur ref head; next ref self
				-- IV  placement/update into middle          => cur ref head; next ref cur.ref
				if valid_a(index) = '0' then
					if empty then
						meta_a_lru(index) <= pol_way;
					else
						meta_a_lru(index) <= head;
					end if;
				else
					if meta_a_lru(index) = pol_way then
						meta_a_lru(index_next) <= pol_way;
					else
						meta_a_lru(index_next) <= meta_a_lru(index);
					end if;
					meta_a_lru(index) <= head;
				end if;
			end if;
		end process state;
	end generate policy_lru;
	
end architecture behaviour;

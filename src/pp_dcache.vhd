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
use work.tracing_types.all;

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
		HAS_MRU          : boolean                       := true;
		HAS_FIFO         : boolean                       := true;
		HAS_RANDOM       : boolean                       := true;
		
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
		crtl  : in std_logic_vector(4 downto 0); -- bit 0 lru; bit 1 dlfu; bit 2 mru; bit 3 fifo; bit 4 rand

		-- Wishbone interface:
		wb_inputs  : in wishbone_master_inputs;
		wb_outputs : out wishbone_master_outputs;
		
		-- trace
		replace_event : out std_logic;
		replace_pol   : out policy_t;
		
		-- Random input
		random : in std_logic_vector(7 downto 0)
	);
end entity pp_dcache;

architecture behaviour of pp_dcache is

	-- address splitting constants
	constant boffset_bits : integer := 2;
	constant region_bits  : integer := 32 - REGION_LD_LEN;
	constant woffset_bits : integer := log2(MAX_LINE_SIZE);
	constant index_bits   : integer := log2(CACHE_DEPTH) - log2(WAYNESS);
	constant tag_bits     : integer := 32 - region_bits - boffset_bits - woffset_bits - index_bits;
	
	-- named values
	constant policy_number : integer := 5;

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
	
	type lru_meta_t is array (0 to way_t'high) of std_logic_vector(way_t'high downto 0);
	
	subtype fifo_meta_t is way_t;
	
	type cache_meta_t is record
		dirty : std_logic;
	end record;
	
	type cache_dlfu_meta_a_line_c  is array(0 to index_t'high)              of dlfu_line_c;
	type cache_dlfu_meta_a_group_c is array(0 to CACHE_DEPTH / WAYNESS - 1) of dlfu_group_c;
	type cache_lru_meta_a          is array(0 to CACHE_DEPTH / WAYNESS - 1) of lru_meta_t;
	type cache_fifo_meta_a         is array(0 to CACHE_DEPTH / WAYNESS - 1) of fifo_meta_t;
	type cache_meta_a              is array(0 to index_t'high)              of cache_meta_t;
	
	-- policy adaptivity
	subtype pol_adapt_ptr_t is integer range 0 to ADAPTIVE_HISTORY - 1;
	
	type pol_adapt_evict_r is array(0 to pol_adapt_ptr_t'high) of cache_tag_t;
	type pol_adapt_pol_r is record
		history    : pol_adapt_evict_r;
		refetched  : std_logic_vector(0 to pol_adapt_ptr_t'high);
		nxt        : pol_adapt_ptr_t;
	end record;
	type pol_adapt_r is array(0 to policy_number - 1) of pol_adapt_pol_r;
	signal pol_adapt : pol_adapt_r;
	
	type eviction_note_t is array(0 to policy_number - 1) of cache_tag_t;
	signal pol_eviction_tag   : eviction_note_t;
	signal pol_eviction_valid : std_logic_vector(0 to policy_number - 1);
	
	subtype pol_prio_t is integer range 0 to ADAPTIVE_HISTORY;
	type pol_prio_a is array(0 to policy_number - 1) of pol_prio_t;

	-- cache memories
	signal data_a  : cache_line_a;
	signal tag_a   : cache_tag_a;
	signal valid_a : std_logic_vector(index_t'high downto 0);
	
	signal meta_a              : cache_meta_a;
	signal meta_a_lru          : cache_lru_meta_a;
	signal meta_a_dlfu_line_c  : cache_dlfu_meta_a_line_c;
	signal meta_a_dlfu_group_c : cache_dlfu_meta_a_group_c;
	signal meta_a_fifo         : cache_fifo_meta_a;
	
	signal lru_meta : lru_meta_t;

	attribute ram_style           : string;
	attribute ram_style of data_a : signal is "block";
	attribute ram_style of tag_a  : signal is "distributed";

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
	signal pol_tag    : addr_tag_t;
	signal pol_empty  : boolean;

	-- controller signals
	type main_state_t is (IDLE, LOOKUP,
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
	signal dlfu_repl, lru_repl, mru_repl, fifo_repl, rand_repl: way_t;
	
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
	
	data_operator: process(clk) begin
		if rising_edge(clk) then
			rdata <= data_a(aindex);
			
			for ii in 0 to MAX_LINE_SIZE * 4 - 1 loop
				if wmask(ii) = '1' then
					data_a(aindex)(8 * (ii + 1) - 1 downto 8 * ii) <= wdata(8 * (ii + 1) - 1 downto 8 * ii);
				end if;
			end loop;
		end if;
	end process data_operator;

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
								else
									main_state <= LOOKUP;
								end if;
							end if;
						when LOOKUP =>
							if mem_read_req = '1' then
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
	pol_tag     <= addr_tag_l;
	pol_way     <= repl_way when main_state = REPLACE else hit_way;
	
	policy_overview: process(valid_a, pol_index) begin
			pol_empty <= true;
			for ii in 0 to way_t'high loop
				if valid_a(get_index(pol_index, ii)) = '1' then
					pol_empty <= false;
				end if;
			end loop;
	end process policy_overview;
	
	policy_sel: process(crtl, dlfu_repl, lru_repl, mru_repl, fifo_repl, rand_repl, pol_adapt, valid_a, pol_index, pol_replace)
		variable prios : pol_prio_a;
		variable en : std_logic_vector(prios'range);
		
		variable empty_slot    : boolean;
		variable empty_slot_nr : way_t;
		
		variable ready : std_logic;
		variable way   : way_t;
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
		for ii in 0 to way_t'high loop
			if not empty_slot and valid_a(get_index(pol_index, ii)) = '0' then
				empty_slot := true;
				empty_slot_nr := ii;
			end if;
		end loop;
		
		for pid in prios'range loop
			if en(pid) = '1' then
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
			
			replace_event <= pol_replace;
			replace_pol.active <= POL_MRU;
		elsif
				en(0) = '1' and
				(prios(0) < prios(1) or en(1) = '0') and
				(prios(0) < prios(3) or en(3) = '0') and
				(prios(0) < prios(4) or en(4) = '0') then
			-- lru is best in recent history
			ready := '1';
			way   := lru_repl;
			
			replace_event <= pol_replace;
			replace_pol.active <= POL_LRU;
		elsif
				en(1) = '1' and
				(prios(1) < prios(3) or en(3) = '0') and
				(prios(1) < prios(4) or en(4) = '0') then
			-- dlfu is best in recent history
			ready := '1';
			way   := dlfu_repl;
			
			replace_event <= pol_replace;
			replace_pol.active <= POL_DLFU;
		elsif
				en(3) = '1' and
				(prios(3) < prios(4) or en(4) = '0') then
			-- fifo is best in recent history
			ready := '1';
			way   := fifo_repl;
			
			replace_event <= pol_replace;
			replace_pol.active <= POL_FIFO;
		elsif
				en(4) = '1' then
			-- rand is best in recent history
			ready := '1';
			way   := rand_repl;
			
			replace_event <= pol_replace;
			replace_pol.active <= POL_RANDOM;
		else
			ready := '1';
			way   := 0;
			
			replace_event <= '0';
			replace_pol.active <= POL_NONE;
		end if;
		
		if empty_slot then
			repl_way     <= empty_slot_nr;
			pol_finished <= '1';
		else
			repl_way     <= way;
			pol_finished <= ready;	
		end if;
	end process policy_sel;
	
	rand_repl <= to_integer(unsigned(random)) mod WAYNESS when HAS_RANDOM else 0;
	
	pol_eviction_tag(0)   <= tag_a  (get_index(pol_index, lru_repl));
	pol_eviction_valid(0) <= valid_a(get_index(pol_index, lru_repl));
	pol_eviction_tag(1)   <= tag_a  (get_index(pol_index, dlfu_repl));
	pol_eviction_valid(1) <= valid_a(get_index(pol_index, dlfu_repl));
	pol_eviction_tag(2)   <= tag_a  (get_index(pol_index, mru_repl));
	pol_eviction_valid(2) <= valid_a(get_index(pol_index, mru_repl));
	pol_eviction_tag(3)   <= tag_a  (get_index(pol_index, fifo_repl));
	pol_eviction_valid(3) <= valid_a(get_index(pol_index, fifo_repl));
	pol_eviction_tag(4)   <= tag_a  (get_index(pol_index, rand_repl));
	pol_eviction_valid(4) <= valid_a(get_index(pol_index, rand_repl));
	
	policy_sel_st: if HAS_DECAYING_LFU and HAS_LRU generate
		process(clk) -- management for all pol-adaptivity state
			variable nxt : pol_adapt_ptr_t;
		begin
			for pid in pol_eviction_tag'range loop
				if rising_edge(clk) then
					nxt := pol_adapt(pid).nxt;
					if reset = '1' then
						pol_adapt(pid).refetched <= (others => '1'); -- init with all ones, new entries will start with 0
						pol_adapt(pid).nxt <= 0;
					elsif pol_eviction_valid(pid) = '1' then -- if line was not valid ignore
						if wb_end_tick = '1' then -- need to update active history
							for ii in 0 to pol_adapt_ptr_t'high loop
								-- three cases:
								-- I   write-out/read-in of line in history     -> mark as refetched
								-- II  write-out/read-in of line not in history -> do nothing
								-- III access to line in history                -> mark as refetched (would have caused refetch if followed)
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
					elsif pol_update = '1' then
						for ii in 0 to pol_adapt_ptr_t'high loop
							if pol_tag = pol_adapt(pid).history(ii) then
								pol_adapt(pid).refetched(ii) <= '1';
							end if;
						end loop;
					end if;
				end if;
			end loop;
		end process;
	end generate policy_sel_st;
	
	policy_dlfu: if HAS_DECAYING_LFU generate
		eval: process(pol_index, meta_a_dlfu_line_c)
			variable index : index_t;
			variable key   : way_t;
			variable min   : dlfu_line_c;
		begin
			key := 0;
			min := dlfu_line_c'high;
			
			for ii in 0 to way_t'high loop
				index := get_index(pol_index, ii);
				
				if meta_a_dlfu_line_c(index) < min then
					min := meta_a_dlfu_line_c(index);
					key := ii;
				end if;
			end loop;
			
			dlfu_repl <= key;
		end process eval;
		
		state: process(clk)
			variable index : index_t;
		begin
			if rising_edge(clk) then
				if pol_update = '1' or pol_replace = '1' then
--					for ii in 0 to way_t'high loop -- group counter mngt
						index := get_index(pol_index, 0); -- no loop as group_c is equal across physical sets
						-- three cases:
						-- I   empty          => 0 -> counter
						-- II  counter at max => 0 -> counter
						-- III middle         => increment counter
						if pol_empty or meta_a_dlfu_group_c(to_integer(unsigned(pol_index))) = dlfu_group_c'high then
							meta_a_dlfu_group_c(to_integer(unsigned(pol_index))) <= 0;
						else
							meta_a_dlfu_group_c(to_integer(unsigned(pol_index))) <= meta_a_dlfu_group_c(to_integer(unsigned(pol_index))) + 1;
						end if;
--					end loop;
					
					for ii in 0 to way_t'high loop -- line counter loop
						index := get_index(pol_index, ii);
						-- five cases:
						-- I   replacement of element          => 0 -> counter
						-- II  access to element without decay => increment counter
						-- III access to element with decay    => (counter >> 1) + 1 -> counter
						-- IV  access to group without decay   => do nothing
						-- V   access to group with decay      => counter >> 1 -> counter
						if pol_way = ii and pol_replace = '1' then -- replace
							meta_a_dlfu_line_c(index) <= 0; -- I
						elsif meta_a_dlfu_group_c(to_integer(unsigned(pol_index))) = dlfu_group_c'high then -- decay
							if pol_way = ii then -- access to this element
								meta_a_dlfu_line_c(index) <= (meta_a_dlfu_line_c(index) / 2) + 1; -- III
							else -- access to group
								meta_a_dlfu_line_c(index) <= meta_a_dlfu_line_c(index) / 2; -- V
							end if;
						else -- no decay
							if pol_way = ii then -- access to this element
								meta_a_dlfu_line_c(index) <= meta_a_dlfu_line_c(index) + 1; -- II
							end if;
						end if;
					end loop;
				end if;
			end if;
		end process state;
	end generate policy_dlfu;
	
	lookup_lru_mru: if HAS_LRU or HAS_MRU generate
		process (pol_index, meta_a_lru) begin
			lru_meta <= meta_a_lru(to_integer(unsigned(pol_index)));
		end process;
	end generate lookup_lru_mru;
	
	policy_lru: if HAS_LRU generate
		eval: process(lru_meta)
			variable key   : way_t;
			variable hit   : boolean;
		begin
			key := 0;
			hit := false;
			
			for ii in 0 to way_t'high loop
				if not hit and lru_meta(ii)(way_t'high) = '1' then
					key := ii;
				end if;
			end loop;
			
			lru_repl <= key;
		end process eval;
	end generate policy_lru;
	
	policy_mru: if HAS_LRU generate
		eval: process(lru_meta)
			variable key   : way_t;
			variable hit   : boolean;
		begin
			key := 0;
			hit := false;
			
			for ii in 0 to way_t'high loop
				if not hit and lru_meta(ii)(0) = '1' then
					key := ii;
				end if;
			end loop;
			
			mru_repl <= key;
		end process eval;
	end generate policy_mru;
	
	policy_lru_mru_state: if HAS_LRU or HAS_MRU generate
		state: process(clk)
			variable repl, this : way_t;
			variable val_repl, val_this : boolean;
		begin
			for ii in 0 to way_t'high loop -- forall ways
				if rising_edge(clk) then
					if pol_empty then
						if pol_way = ii then
							meta_a_lru(to_integer(unsigned(pol_index))) <= (ii => (0 => '1', others => '0'), others => (others => '0'));
						end if;
					else
						-- two cases
						-- I  placement/access into this                   => record to most recently accessed
						-- II placement/access into less recently accessed => shift

						if pol_way = ii then -- I
							meta_a_lru(to_integer(unsigned(pol_index)))(ii) <= (0 => '1', others => '0');
						else -- II prep for test
							val_repl := false;
							val_this := false;
							repl := 0;
							this := 0;
							for jj in 0 to way_t'high loop
								if not val_repl and lru_meta(pol_way)(jj) = '1' then -- find age of the replaced
									repl := jj;
									val_repl := true;
								end if;
								
								if not val_this and lru_meta(ii)(jj) = '1' then -- find age of this
									this := jj;
									val_this := true;
								end if;
							end loop;
							
							if val_repl and val_this and repl > this then -- test for II
								meta_a_lru(to_integer(unsigned(pol_index)))(ii) <= std_logic_vector(shift_left(unsigned(
									meta_a_lru(to_integer(unsigned(pol_index)))(ii)
								), 1));
							end if;
						end if;
					end if;
				end if;
			end loop;
		end process state;
	end generate policy_lru_mru_state;
	
	policy_fifo: if HAS_FIFO generate
		eval: process(meta_a_fifo, pol_index) begin
			fifo_repl <= meta_a_fifo(to_integer(unsigned(pol_index)));
		end process eval;
		
		state: process(clk) begin
			if rising_edge(clk) then
				if pol_empty then
					meta_a_fifo(to_integer(unsigned(pol_index))) <= 0;
				elsif pol_replace = '1' then
					-- one case
					-- placement into anywhere => increment counter
					meta_a_fifo(to_integer(unsigned(pol_index))) <= (meta_a_fifo(to_integer(unsigned(pol_index))) + 1) mod WAYNESS;
				end if;
			end if;
		end process state;
	end generate policy_fifo;
	
end architecture behaviour;

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
		DLFU_RATE        : integer                       := 32           --! Number of accesses to a set between decays.
	);
	port(
		clk   : in std_logic;
		reset : in std_logic;

		-- Processor data memory signals:
		signal mem_address   : in  std_logic_vector(31 downto 0);
		signal mem_data_in   : in  std_logic_vector(31 downto 0); -- Data in from the bus
		signal mem_data_out  : out std_logic_vector(31 downto 0); -- Data out to the bus
		signal mem_data_size : in  std_logic_vector( 1 downto 0);
		signal mem_read_req  : in  std_logic;
		signal mem_read_ack  : out std_logic;
		signal mem_write_req : in  std_logic;
		signal mem_write_ack : out std_logic;

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
	subtype index_t  is integer range 0 to CACHE_DEPTH;
	subtype way_t    is integer range 0 to WAYNESS;
	subtype offset_t is integer range 0 to MAX_LINE_SIZE;

	subtype word_mask_t is std_logic_vector(3 downto 0);
	type word_mask_a    is array(0 to MAX_LINE_SIZE - 1) of word_mask_t;
	subtype line_mask_t is std_logic_vector(MAX_LINE_SIZE * 4 - 1 downto 0);
	
	-- cache line types
	subtype cache_line_t    is std_logic_vector(MAX_LINE_SIZE * 32 - 1 downto 0);
	type cache_line_words_a is array(0 to MAX_LINE_SIZE - 1) of std_logic_vector(31 downto 0);
	type cache_line_a       is array(0 to CACHE_DEPTH - 1) of cache_line_t;

	-- cache tag type
	subtype cache_tag_t is addr_tag_t;
	type cache_tag_a    is array(0 to CACHE_DEPTH - 1) of cache_tag_t;

	-- cache metadata type
	type dlfu_meta_t is record
		set_c  : integer range 0 to DLFU_RATE;
		line_c : integer range 0 to DLFU_RATE * 2;
	end record;
	type cache_meta_t is record
		dirty : std_logic;
		lru   : integer range 0 to WAYNESS;
		dlfu  : dlfu_meta_t;
	end record;
	type cache_meta_a is array(0 to CACHE_DEPTH - 1) of cache_meta_t;

	-- cache memories
	signal data_a  : cache_line_a;
	signal tag_a   : cache_tag_a;
	signal meta_a  : cache_meta_a;
	signal valid_a : std_logic_vector(CACHE_DEPTH - 1 downto 0);

	attribute ram_style            : string;
	attribute ram_style of data_a  : signal is "block";

	-- signals for response control
	signal cache_hit  : std_logic;
	signal hit_way    : way_t;
	signal hit_line_shifted : cache_line_t;
	signal hit_line_a : cache_line_words_a;
	signal pt_was_we  : std_logic;

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
	signal line_index : index_t;
	signal lookup_tag : addr_tag_t;
	signal access_mask, access_mask_shifted : line_mask_t;
	signal data, data_shifted : cache_line_t;

	signal pol_finished : std_logic;
	signal wb_start, wb_stop : offset_t;
	signal wb_rbuffer_words, wb_wbuffer_words : cache_line_words_a;
	signal wb_rbuffer_line, wb_wbuffer_line : cache_line_t;
	signal wb_wtag : addr_tag_t;

	type wb_mode_t is (IDLE, WRITE, READ, WRITE_WAIT, READ_WAIT);
	signal wb_mode : wb_mode_t;
	signal wb_tag  : addr_tag_t;
	signal wb_mask : line_mask_t;
	signal wb_mask_a : word_mask_a;

	-- general policy signals
	type active_pol_t is (DLFU, LRU);
	signal active_pol : active_pol_t;

	signal pol_update, normal_cycle, pol_replace, resp_rdata : std_logic;
	signal pol_index : addr_index_t;
	signal pol_way   : way_t;

	-- dlfu policy signals
	signal dlfu_repl : way_t;
	
	-- block ram operator signals
	signal rdata, wdata : cache_line_t;
	signal wmask  : line_mask_t;
	signal aindex : index_t;

	-- helper signal
	signal index_h, index_r : index_t;

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

	mem_data_out <= std_logic_vector(shift_right(unsigned(hit_line_a(to_integer(unsigned(addr_woffset_l)))), 8 * to_integer(unsigned(addr_boffset_l))));

	in_segment <= '1' when addr_region = REGION_BASE(31 downto 32 - region_bits) else '0';
	line_index <= get_index(addr_index, hit_way);

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

	controller: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				main_state <= IDLE;
				wb_mode <= IDLE;
				wmask <= (others => '0');
				valid_a <= (others => '0');
				normal_cycle <= '1';
				resp_rdata   <= '0';
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
								if wb_mode = WRITE
								then wb_mode <= WRITE_WAIT;
								else wb_mode <= READ_WAIT;
								end if;
								wb_start <= wb_start + 1;
							end if;
							wb_rbuffer_words(wb_start) <= wb_inputs.dat;
						end if;
				end case;

				case main_state is
					when IDLE =>
						normal_cycle  <= '1';
						resp_rdata    <= '0';
						mem_read_ack  <= '0';
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
							mem_read_ack <= '1';
							resp_rdata   <= '1';
							main_state <= IDLE;
						end if;
					when WRITE_BACK =>
						if wb_mode = IDLE then
							mem_read_ack <= '1';
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
							meta_a (index_r).dirty <= '0';
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
						mem_read_ack  <= not pt_was_we;
						mem_write_ack <= pt_was_we;
						main_state <= IDLE;
				end case;
			end if;
		end if;
	end process controller;

	pol_update  <= normal_cycle when main_state = READ_RESPOND or main_state = WRITE_RESPOND else '0';
	pol_replace <= '1' when main_state = REPLACE else '0';
	pol_index   <= addr_index_l;
	pol_way     <= repl_way when main_state = REPLACE else hit_way;

	active_pol <= LRU;
	
	policy_sel: process(active_pol, dlfu_repl) begin
		case active_pol is
			when DLFU =>
				repl_way <= dlfu_repl;
				pol_finished <= '1';
			when others =>
				repl_way <= 0;
				pol_finished <= '1';
		end case;
	end process policy_sel;
	
end architecture behaviour;

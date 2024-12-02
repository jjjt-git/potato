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
		DLFU_RATE        : integer
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
	constant region_bits : integer := 32 - REGION_LD_LEN;
	constant offset_bits : integer := log2(MAX_LINE_SIZE);
	constant index_bits  : integer := log2(CACHE_DEPTH) - log2(WAYNESS);
	constant tag_bits    : integer := 32 - region_bits - offset_bits - index_bits;

	-- address part types
	subtype addr_region_t is std_logic_vector(region_bits - 1 downto 0);
	subtype addr_offset_t is std_logic_vector(offset_bits - 1 downto 0);
	subtype addr_index_t  is std_logic_vector(index_bits  - 1 downto 0);
	subtype addr_tag_t    is std_logic_vector(tag_bits    - 1 downto 0);

	subtype addr_line_t is std_logic_vector(31 downto offset_bits);

	-- selection types
	subtype index_t  is integer range 0 to CACHE_DEPTH;
	subtype way_t    is integer range 0 to WAYNESS;
	subtype offset_t is integer range 0 to MAX_LINE_SIZE;

	subtype word_mask_t is std_logic_vector(3 downto 0);
	
	-- cache line types
	subtype cache_line_t    is std_logic_vector(MAX_LINE_SIZE * 32 - 1 downto 0);
	type cache_line_words_a is array(0 to MAX_LINE_SIZE - 1) of std_logic_vector(31 downto 0);
	type cache_line_a       is array(0 to CACHE_DEPTH - 1) of cache_line_t;

	-- cache tag type
	subtype cache_tag_t is addr_tag_t;
	type cache_tag_a    is array(0 to CACHE_DEPTH - 1) of cache_tag_t;

	-- cache metadata type
	type cache_meta_t is record
		dirty : std_logic;
		lru : std_logic_vector(log2(WAYNESS) - 1 downto 0);
	end record;
	type cache_meta_a is array(0 to CACHE_DEPTH - 1) of cache_meta_t;

	-- cache memories
	signal data_a  : cache_line_a;
	signal tag_a   : cache_tag_a;
	signal meta_a  : cache_meta_a;
	signal valid_a : std_logic_vector(CACHE_DEPTH - 1 downto 0);

	attribute ram_style           : string;
	attribute ram_style of data_a : signal is "block";

	-- signals for response control
	signal cache_hit  : std_logic;
	signal hit_way    : way_t;
	signal hit_line   : cache_line_t;
	signal hit_line_a : cache_line_words_a;

	-- address split signals
	signal addr_region : addr_region_t;
	signal addr_offset : addr_offset_t;
	signal addr_index  : addr_index_t;
	signal addr_tag    : addr_tag_t;

	-- signals for replace control
	signal repl_way   : way_t;

	-- controller signals
	type main_state_t is (IDLE,
		PASS_THROUGH, PASS_THROUGH_RESPOND,
		WRITE_RESPOND,
		READ_RESPOND, REFILL, REPLACE, WRITE_BACK);
	signal main_state : main_state_t;

	signal in_segment, need_wb : std_logic;
	signal line_index : index_t;
	signal lookup_tag : addr_tag_t;
	signal access_mask, access_mask_shifted : word_mask_t;
	signal data_shifted : std_logic_vector(31 downto 0);

	signal pol_finished : std_logic;
	signal wb_start, wb_stop : offset_t;
	signal wb_rbuffer_words, wb_wbuffer_words : cache_line_words_a;
	signal wb_rbuffer_line, wb_wbuffer_line : cache_line_t;
	signal wb_wtag : addr_tag_t;

	type wb_mode_t is (IDLE, WRITE, READ);
	signal wb_mode : wb_mode_t;
	signal wb_tag  : addr_tag_t;
	signal wb_mask : word_mask_t;

	-- helper functions
	function get_index(
		entry : addr_index_t;
		way   : way_t
	) return index_t is begin
		return to_integer(unsigned(entry)) * WAYNESS + way;
	end function;

	function get_address(
		entry : addr_index_t;
		tag   : addr_tag_t
	) return addr_line_t is begin
		return REGION_BASE(31 downto REGION_LD_LEN) & tag & entry;
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

begin
	assert is_pow2(MAX_LINE_SIZE) report "Cache line size must be a power of 2!" severity FAILURE;
	assert is_pow2(CACHE_DEPTH) report "Cache depth must be a power of 2!" severity FAILURE;
	assert is_pow2(WAYNESS) report "Wayness must be a power of 2!" severity FAILURE;

	-- splitting of address
	-----------------------------------
	-- region | tag | index | offset --
	-----------------------------------
	addr_region <= mem_address(31 downto 32 - region_bits);
	addr_tag    <= mem_address(31 - region_bits downto 32 - region_bits - tag_bits);
	addr_index  <= mem_address(31 - region_bits - tag_bits downto 32 - region_bits - tag_bits - index_bits);
	addr_offset <= mem_address(31 - region_bits - tag_bits - index_bits downto 32 - region_bits - tag_bits - index_bits - offset_bits);

	mem_data_out <= hit_line_a(to_integer(unsigned(addr_offset)));

	ack_signals: process(main_state) begin
		case main_state is
			when READ_RESPOND =>
				mem_read_ack <= '1';
				mem_write_ack <= '0';
			when WRITE_RESPOND =>
				mem_read_ack <= '0';
				mem_write_ack <= '1';
			when others =>
				mem_read_ack <= '0';
				mem_write_ack <= '0';
		end case;
	end process ack_signals;

	in_segment <= '1' when addr_region = REGION_BASE(31 downto 32 - region_bits) else '0';
	line_index <= get_index(addr_index, hit_way);

	access_mask <= get_mask(mem_data_size);
	access_mask_shifted <= access_mask sll to_integer(unsigned(mem_address(1 downto 0)));
	data_shifted <= mem_data_in sll (to_integer(unsigned(mem_address(1 downto 0))) * 8);

	decompose_line: for ii in 0 to MAX_LINE_SIZE - 1 generate
		hit_line_a(ii) <= hit_line(32 * ii + 31 downto 32 * ii);
	end generate decompose_line;

	tag_lookup: process(addr_tag, addr_index, tag_a, valid_a) begin
		for ii in 0 to WAYNESS loop
			if valid_a(get_index(addr_index, ii)) = '1' and tag_a(get_index(addr_index, ii)) = addr_tag then
				hit_way <= ii;
			end if;
		end loop;
	end process tag_lookup;

	wb_outputs.sel <= wb_mask;

	wb_outputs.adr <= get_address(addr_index, wb_tag) & std_logic_vector(to_unsigned(wb_start, addr_offset'length));
	wb_outputs.dat <= wb_wbuffer_words(wb_start);

	wb_signals: process(wb_mode) begin
		case wb_mode is
			when WRITE =>
				wb_outputs.we  <= '1';
				wb_outputs.cyc <= '1';
				wb_outputs.stb <= '1';
			when READ =>
				wb_outputs.we  <= '0';
				wb_outputs.cyc <= '1';
				wb_outputs.stb <= '1';
			when others =>
				wb_outputs.we  <= '0';
				wb_outputs.cyc <= '0';
				wb_outputs.stb <= '0';
		end case;
	end process wb_signals;

	controller: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				main_state <= IDLE;
				wb_mode <= IDLE;
			else
				case wb_mode is
					when IDLE => wb_mode <= IDLE;
					when others =>
						if wb_inputs.ack = '1' then
							if wb_start = wb_stop then
								wb_mode <= IDLE;
							else
								wb_start <= wb_start + 1;
								wb_rbuffer_words(wb_start) <= wb_inputs.dat;
							end if;
						end if;
				end case;

				case main_state is
					when IDLE =>
						if mem_read_req or mem_write_req then -- pseudo-transition to addr-decode
							if in_segment = '0' then
								main_state <= PASS_THROUGH;
							elsif mem_read_req then
								need_wb <= '0';
								if cache_hit then
									main_state <= READ_RESPOND;
								else
									wb_tag <= addr_tag;

									wb_start <= 0;
									wb_stop  <= MAX_LINE_SIZE - 1;
									wb_mask  <= b"1111";
									wb_mode  <= READ;

									main_state <= REFILL;
								end if;
							else
							if cache_hit then
								main_state <= WRITE_RESPOND;
								else
								end if;
							end if;
						end if;

					when READ_RESPOND =>
						if need_wb then
							main_state <= WRITE_BACK;
						else
							main_state <= IDLE;
						end if;
					when WRITE_BACK =>
						if wb_mode = IDLE then
							main_state <= IDLE;
						end if;
					when REFILL =>
						if wb_mode = IDLE and pol_finished = '1' then
							main_state <= REPLACE;
						end if;
					when REPLACE =>
						if meta_a(get_index(addr_index, repl_way)).dirty then
							wb_tag <= tag_a(get_index(addr_index, repl_way));
							wb_wbuffer_line <= data_a(get_index(addr_index, repl_way));
	
							wb_start <= 0;
							wb_stop  <= MAX_LINE_SIZE - 1;
							wb_mask  <= b"1111";
							wb_mode  <= WRITE;
						end if;

						need_wb <= meta_a(get_index(addr_index, repl_way)).dirty;
						
						tag_a (get_index(addr_index, repl_way)) <= addr_tag;
						data_a(get_index(addr_index, repl_way)) <= wb_rbuffer_line;
						meta_a(get_index(addr_index, repl_way)).dirty <= '0';

						main_state <= READ_RESPOND;

					when WRITE_RESPOND =>
						if access_mask_shifted(0) = '1' then
							data_a(get_index(addr_index, hit_way))(07 downto 00) <= data_shifted(07 downto 00);
						end if;
						if access_mask_shifted(1) = '1' then
							data_a(get_index(addr_index, hit_way))(15 downto 08) <= data_shifted(15 downto 08);
						end if;
						if access_mask_shifted(2) = '1' then
							data_a(get_index(addr_index, hit_way))(23 downto 16) <= data_shifted(23 downto 16);
						end if;
						if access_mask_shifted(3) = '1' then
							data_a(get_index(addr_index, hit_way))(31 downto 24) <= data_shifted(31 downto 24);
						end if;

						main_state <= IDLE;

					when PASS_THROUGH =>
						wb_start <= to_integer(unsigned(addr_offset));
						wb_stop  <= to_integer(unsigned(addr_offset));
						wb_tag   <= addr_tag;
						wb_mask  <= access_mask_shifted;

						wb_wbuffer_line <= data_shifted sll (to_integer(unsigned(addr_offset)) * 32);

						if mem_read_req then
							wb_mode <= READ;
						else
							wb_mode <= WRITE;
						end if;

						main_state <= PASS_THROUGH_RESPOND;
					when PASS_THROUGH_RESPOND =>
				end case;
			end if;
		end if;
	end process controller;
	
end architecture behaviour;

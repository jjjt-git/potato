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
		
		cache_miss, cache_r_miss : out std_logic;

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
	constant enable_debug : boolean := false;

	-- address splitting constants
	constant boffset_bits : integer := 2;
	constant region_bits  : integer := 32 - REGION_LD_LEN;
	constant offset_bits  : integer := log2(MAX_LINE_SIZE);
	constant index_bits   : integer := log2(CACHE_DEPTH);
	constant tag_bits     : integer := 32 - region_bits - boffset_bits - offset_bits - index_bits;
	
	subtype word_t     is std_logic_vector(31 downto 0);
	subtype wordmask_t is std_logic_vector( 3 downto 0);
	subtype region_t   is std_logic_vector(region_bits - 1 downto 0);
	subtype tag_t      is std_logic_vector(   tag_bits - 1 downto 0);
	subtype offset_t   is integer range 0 to MAX_LINE_SIZE - 1;
	subtype index_t    is integer range 0 to CACHE_DEPTH - 1;
	subtype way_t      is integer range 0 to WAYNESS - 1;
	
	signal addr_region, latch_region : region_t;
	signal addr_offset, latch_offset : offset_t;
	signal addr_index,  latch_index  : index_t;
	signal latch_tag   : tag_t;
	
	signal addr_latch : std_logic_vector(31 downto 0);
	signal size_latch : std_logic_vector( 1 downto 0);
	
	signal resetting : boolean;
	signal reset_ctr : index_t;
	
	subtype cache_line_t is std_logic_vector(MAX_LINE_SIZE * 32 - 1 downto 0);
	type cache_line_decomp_t is array(offset_t) of word_t;
	type cache_entry_t is record
		data  : cache_line_t;
		tag   : tag_t;
		dirty : boolean;
	end record;
	type cache_entry_w_t is array(way_t) of cache_entry_t;
	subtype cache_entry_w_physical_t is std_logic_vector(WAYNESS * (cache_line_t'length + tag_t'length + 1) - 1 downto 0);
	
	subtype cache_entry_v_t is std_logic_vector(way_t'high downto 0);
	
	type cache_a_t is array(index_t) of cache_entry_w_physical_t;
	type valid_a_t is array(index_t) of cache_entry_v_t;
	
	signal cache_a_out, cache_a_in : cache_entry_w_physical_t;
	
	signal cache_a : cache_a_t;
	signal valid_a : valid_a_t;
	
	signal array_en, array_we : boolean;
	signal bus_start_wr, bus_start_re : boolean;
	signal bus_start_word, bus_end_word : offset_t;
	
	attribute ram_style            : string;
	attribute ram_style of cache_a : signal is "block";
	attribute ram_style of valid_a : signal is "distributed";
	
	type cache_crtl_state_t is (
		IDLE, LOOKUP,
		PASS_THROUGH,
		WRITE_RESPOND, READ_RESPOND,
		REFILL, REPLACE, UPDATE_REFILL, WRITE_BACK
	);
	type bus_crtl_state_t is (IDLE, WRITE, READ);
	
	signal cache_crtl : cache_crtl_state_t;
	signal bus_crtl   : bus_crtl_state_t;
	
	signal bus_start, bus_end : offset_t;
	signal bus_finished : std_logic;
	
	signal in_segment, hit : boolean;
	signal hit_way : way_t;
	signal current_entry, current_entry_update : cache_entry_w_t;
	signal current_valid, current_valid_update : cache_entry_v_t;
	
	signal proc_word_in : word_t;
	signal bus_block_in,   bus_block_out   : cache_line_t;
	signal bus_block_in_a, bus_block_out_a : cache_line_decomp_t;
	
	-- policy interaction signals
	signal pol_finished, pol_replace, pol_update : std_logic;
	signal pol_victim, pol_way : way_t;
	signal pol_tag     : tag_t;
	signal pol_index   : index_t;
	signal pol_tag_a   : std_logic_vector(tag_bits * WAYNESS - 1 downto 0);
	signal pol_valid_a : cache_entry_v_t;
	
	-- debug
	attribute MARK_DEBUG : boolean;
	attribute MARK_DEBUG of hit          : signal is enable_debug;
	attribute MARK_DEBUG of hit_way      : signal is enable_debug;
	attribute MARK_DEBUG of in_segment   : signal is enable_debug;
	attribute MARK_DEBUG of pol_update   : signal is enable_debug;
	attribute MARK_DEBUG of pol_replace  : signal is enable_debug;
	attribute MARK_DEBUG of current_valid: signal is enable_debug;
	attribute MARK_DEBUG of current_entry: signal is enable_debug;
	attribute MARK_DEBUG of current_valid_update: signal is enable_debug;
	attribute MARK_DEBUG of current_entry_update: signal is enable_debug;
	attribute MARK_DEBUG of cache_crtl   : signal is enable_debug;
	
	-- copied from pp_wb_adapter
	function get_data_shift(
		size    : in std_logic_vector(1 downto 0);
		address : in std_logic_vector
	) return natural is
	begin
		case size is
			when b"01" =>
				case address(1 downto 0) is
					when b"00" =>
						return 0;
					when b"01" =>
						return 8;
					when b"10" =>
						return 16;
					when b"11" =>
						return 24;
					when others =>
						return 0;
				end case;
			when b"10" =>
				if address(1) = '0' then
					return 0;
				else
					return 16;
				end if;
			when others =>
				return 0;
		end case;
	end function get_data_shift;
begin
	assert is_pow2(MAX_LINE_SIZE) report "Cache line size must be a power of 2!" severity FAILURE;
	assert is_pow2(CACHE_DEPTH) report "Cache depth must be a power of 2!" severity FAILURE;
	assert is_pow2(WAYNESS) report "Wayness must be a power of 2!" severity FAILURE;
	
	-- splitting of address
	---------------------------------------------------------------------------
	-- region | tag | index | offset | inner word offset (mostly irrelevant) --
	---------------------------------------------------------------------------
	addr_region  <= mem_address(31 downto 32 - region_bits);
	latch_region <= addr_latch (31 downto 32 - region_bits);
	
	latch_tag    <= addr_latch(31 - region_bits downto 32 - region_bits - tag_bits);
	addr_index  <= to_integer(unsigned(mem_address(31 - region_bits - tag_bits              downto 32 - region_bits - tag_bits - index_bits)));
	latch_index  <= to_integer(unsigned(addr_latch(31 - region_bits - tag_bits              downto 32 - region_bits - tag_bits - index_bits)));
	addr_offset  <= to_integer(unsigned(mem_address(31 - region_bits - tag_bits - index_bits downto 32 - region_bits - tag_bits - index_bits - offset_bits)));
	latch_offset <= to_integer(unsigned(addr_latch(31 - region_bits - tag_bits - index_bits downto 32 - region_bits - tag_bits - index_bits - offset_bits)));
	
	assert_constraints: process(clk) begin
		if rising_edge(clk) then
			assert (mem_read_req and mem_write_req) = '0' report "Only one type of request can be issued at one time" severity FAILURE;
			
			if mem_read_req = '1' or mem_write_req = '1' then
				if mem_data_size = b"10" then
					assert mem_address(0) = '0' report "Access must be half word aligned!" severity FAILURE;
				end if;
				
				if mem_data_size = b"11" or mem_data_size = b"00" then
					assert mem_address(1 downto 0) = b"00" report "Access must be word aligned!" severity FAILURE;
				end if;
			end if;
		end if;
	end process assert_constraints;
	
	decompose_block: process(bus_block_out, bus_block_in_a) begin
		for ii in offset_t loop
			bus_block_out_a(ii) <= bus_block_out((ii + 1) * word_t'length - 1 downto ii * word_t'length);
			
			bus_block_in((ii + 1) * word_t'length - 1 downto ii * word_t'length) <= bus_block_in_a(ii);
		end loop;
	end process decompose_block;
	
	check_hit: process(current_entry, current_valid, latch_tag)
		variable found : boolean;
		variable way   : way_t;
	begin
		found := False;
		way   := 0;
		
		for ii in way_t loop
			if current_entry(ii).tag = latch_tag and current_valid(ii) = '1' then
				way   := ii;
				found := True;
			end if;
		end loop;
		
		hit     <= found;
		hit_way <= way;
	end process;
	in_segment <=
		False when global_enable = '0' else
		True  when addr_region = REGION_BASE(31 downto 32 - region_bits) else
		False;
	
	wb_outputs.cyc <= '0' when bus_crtl = IDLE  else '1';
	wb_outputs.stb <= '0' when bus_crtl = IDLE  else '1';
	wb_outputs.we  <= '1' when bus_crtl = WRITE else '0';
	
	wb_outputs.adr <=
			latch_region &
			latch_tag &
			std_logic_vector(to_unsigned(latch_index, index_bits) &
			to_unsigned(bus_start, offset_bits)) &
			b"00";
	wb_outputs.dat <= bus_block_out_a(bus_start);
	
	current_valid <= valid_a(latch_index);
	
	cache_crtl_eval: process(
		cache_crtl, bus_crtl,
		mem_read_req, mem_write_req,
		in_segment, resetting, hit,
		latch_offset, addr_offset,
		current_entry,
		pol_victim
	)
		variable c_en, c_we : boolean;
		variable b_w_start, b_w_end : offset_t;
		variable b_start_read, b_start_write : boolean;
		
		variable is_miss, is_r_miss : std_logic;
	begin
		is_miss   := '0';
		is_r_miss := '0';
	
		c_en := False;
		c_we := False;
		
		b_w_start := 0;
		b_w_end   := 0;
		
		b_start_read  := False;
		b_start_write := False;
					
		if cache_crtl = IDLE then
			if bus_crtl = IDLE and (mem_read_req = '1' or mem_write_req = '1') then
				if in_segment then
					if not resetting then
						c_en := True;
					end if;
				else
					b_w_start := addr_offset;
					b_w_end   := addr_offset;
					
					if mem_read_req = '1' then
						b_start_read  := True;
					else
						b_start_write := True;
					end if;
				end if;
			end if;
		elsif cache_crtl = LOOKUP then
			if not hit then
				is_miss := '1';
				
				if mem_read_req = '1' then
					is_r_miss := '1';
				
					b_start_read := True;
					b_w_start    := latch_offset;
					b_w_end      := (latch_offset + MAX_LINE_SIZE - 1) mod MAX_LINE_SIZE;
				else
					b_start_write := True;
					b_w_start     := latch_offset;
					b_w_end       := latch_offset;
				end if;
			end if;
		elsif cache_crtl = WRITE_RESPOND then
			c_en := True;
			c_we := True;
		elsif cache_crtl = UPDATE_REFILL then
			if current_entry(pol_victim).dirty then
				b_start_write := True;
				b_w_start := 0;
				b_w_end   := MAX_LINE_SIZE - 1;
			end if;
			
			c_en := True;
			c_we := True;
		end if;
		
		cache_miss   <= is_miss;
		cache_r_miss <= is_r_miss;
		
		array_en <= c_en;
		array_we <= c_we;
		
		bus_start_word <= b_w_start;
		bus_end_word   <= b_w_end;
		
		bus_start_re <= b_start_read;
		bus_start_wr <= b_start_write;
	end process cache_crtl_eval;
	
	cache_crtl_st: process(clk)
		variable proc_word: std_logic_vector(31 downto 0);
	begin
		if rising_edge(clk) then
			if reset = '1' then
				cache_crtl <= IDLE;
				
				mem_read_ack   <= '0';
				mem_write_ack  <= '0';
			else
				case cache_crtl is
					when IDLE =>
						mem_read_ack   <= '0';
						mem_write_ack  <= '0';
						
						if bus_crtl = IDLE and (mem_read_req = '1' or mem_write_req = '1') then
							addr_latch <= mem_address;
							size_latch <= mem_data_size;
							proc_word    := std_logic_vector(shift_left (unsigned(mem_data_in), get_data_shift(mem_data_size, mem_address)));
							proc_word_in <= proc_word;
						
							if in_segment then
								if not resetting then -- array is not yet operational, but pass-through is possible
									cache_crtl <= LOOKUP;
								end if;
							else
								cache_crtl <= PASS_THROUGH;
								
								if mem_read_req = '1' then
									wb_outputs.sel <= (others => '1');
								else
									bus_block_out((addr_offset + 1) * word_t'length - 1 downto addr_offset * word_t'length) <= proc_word;
									wb_outputs.sel <= wb_get_data_sel(mem_data_size, mem_address);
								end if;
							end if;
						end if;
					
					when PASS_THROUGH =>
						if mem_read_req = '1' then
							if bus_crtl = IDLE then
								cache_crtl <= IDLE;
								
								mem_read_ack <= '1';
								mem_data_out <= std_logic_vector(
									shift_right(
										unsigned(bus_block_in_a(latch_offset)),
										get_data_shift(size_latch, addr_latch)
									)
								);
							end if;
						else
							cache_crtl <= IDLE;
							
							mem_write_ack <= '1';
						end if;
						
					when LOOKUP =>
						if mem_read_req = '1' then
							if hit then
								cache_crtl <= READ_RESPOND;
							else
								cache_crtl <= REFILL;
								
								wb_outputs.sel <= (others => '1');
							end if;
						else
							if hit then
								cache_crtl <= WRITE_RESPOND;
							else
								cache_crtl <= PASS_THROUGH;
								
								bus_block_out((latch_offset + 1) * word_t'length - 1 downto latch_offset * word_t'length) <= proc_word_in;
								wb_outputs.sel <= wb_get_data_sel(size_latch, addr_latch);
							end if;
						end if;
						
					when WRITE_RESPOND =>
						cache_crtl <= IDLE;
						
						mem_write_ack <= '1';
					
					when READ_RESPOND =>
						cache_crtl <= IDLE;
						
						mem_read_ack <= '1';
						mem_data_out <=
							std_logic_vector(
								shift_right(
									unsigned(current_entry(hit_way).data(
										(latch_offset + 1) * word_t'length - 1 downto latch_offset * word_t'length
									)),
									get_data_shift(size_latch, addr_latch)
								)
							);
						
					when REFILL =>
						if wb_inputs.ack = '1' then -- at least one word was read
							cache_crtl <= REPLACE;
								
							mem_read_ack <= '1';
							mem_data_out <= std_logic_vector(
								shift_right(
									unsigned(wb_inputs.dat),
									get_data_shift(size_latch, addr_latch)
								)
							);
						end if;
						
					when REPLACE =>
						mem_read_ack <= '0';
						if bus_crtl = IDLE and pol_finished = '1' then
							report "Replacing into " & integer'image(pol_victim) severity NOTE;
							cache_crtl <= UPDATE_REFILL;
						end if;
						
					when UPDATE_REFILL =>
						if current_entry(pol_victim).dirty then
							cache_crtl <= WRITE_BACK;
							
							bus_block_out <= current_entry(pol_victim).data;
							
							addr_latch(31 - region_bits downto 32 - region_bits - tag_bits) <= current_entry(pol_victim).tag;
						else
							cache_crtl <= IDLE;
						end if;
						
					when WRITE_BACK =>
						if bus_crtl = IDLE then
							cache_crtl <= IDLE;
						end if;
				end case;
			end if;
		end if;
	end process cache_crtl_st;
	
	bus_crtl_st: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				bus_crtl   <= IDLE;
			else
				assert not (bus_start_re and bus_start_wr) report "Bus CRTL cannot process multiple commands" severity FAILURE;
				if bus_start_re or bus_start_wr then
					assert bus_crtl = IDLE report "Bus CRTL command was issued while active" severity FAILURE;
				
					if bus_start_re then
						bus_crtl <= READ;
					else
						bus_crtl <= WRITE;
					end if;
					bus_start <= bus_start_word;
					bus_end   <= bus_end_word;
				else
					case bus_crtl is
						when IDLE => bus_crtl <= IDLE;
						when others =>
							if wb_inputs.ack = '1' then
								if bus_start = bus_end then
									bus_crtl <= IDLE;
								else
									bus_start <= (bus_start + 1) mod MAX_LINE_SIZE;
								end if;
								bus_block_in_a(bus_start) <= wb_inputs.dat;
							end if;
					end case;
				end if;
			end if;
		end if;
	end process bus_crtl_st;
		
	cache_array_op: process(clk) begin
		if rising_edge(clk) then
			if array_en then
				if array_we then
					cache_a(latch_index) <= cache_a_in;
					cache_a_out          <= cache_a(latch_index);
				else
					cache_a_out          <= cache_a(addr_index);
				end if;
			end if;
		end if;
	end process cache_array_op;
		
	valid_array_op: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				resetting <= True;
				reset_ctr <= 0;
			elsif resetting then
				if reset_ctr = index_t'high then
					resetting <= False;
				else
					reset_ctr <= reset_ctr + 1;
				end if;
				
				valid_a(reset_ctr) <= (others => '0');
			else
				if array_we then
					valid_a(latch_index) <= current_valid_update;
				end if;
			end if;
		end if;
	end process valid_array_op;
	
	entry_to_physical: process(current_entry_update)
		variable work : cache_entry_w_physical_t;
		variable rec  : std_logic_vector(cache_line_t'length + tag_t'length + 1 - 1 downto 0);
	begin
		for ii in way_t loop
			rec(cache_line_t'length + tag_t'length + 1 - 1 downto tag_t'length + 1) := current_entry_update(ii).data;
			rec(tag_t'length + 1 - 1 downto 1) := current_entry_update(ii).tag;
			if current_entry_update(ii).dirty then
				rec(0) := '1';
			else
				rec(0) := '0';
			end if;
			
			work((ii + 1) * (cache_line_t'length + tag_t'length + 1) - 1 downto ii * (cache_line_t'length + tag_t'length + 1)) := rec;
		end loop;
		cache_a_in <= work;
	end process entry_to_physical;
	
	entry_from_physical: process(cache_a_out)
		variable work : cache_entry_w_t;
		variable rec  : std_logic_vector(cache_line_t'length + tag_t'length + 1 - 1 downto 0);
	begin
		for ii in way_t loop
			rec := cache_a_out((ii + 1) * (cache_line_t'length + tag_t'length + 1) - 1 downto ii * (cache_line_t'length + tag_t'length + 1));
			
			work(ii).data  := rec(cache_line_t'length + tag_t'length + 1 - 1 downto tag_t'length + 1);
			work(ii).tag   := rec(tag_t'length + 1 - 1 downto 1);
			if rec(0) = '1' then
				work(ii).dirty := True;
			else
				work(ii).dirty := False;
			end if;
		end loop;
		current_entry <= work;
	end process entry_from_physical;
	
	entry_update: process(
		current_entry, current_valid,
		cache_crtl, bus_crtl,
		proc_word_in, bus_block_in,
		hit_way, pol_victim, latch_offset,
		pol_finished,
		latch_tag, mem_data_size, mem_address
	)
		variable work     : cache_entry_w_t;
		variable work_v   : cache_entry_v_t;
		
		variable work_boffset : cache_line_t'range;
		
		variable data_mask: std_logic_vector(3 downto 0);
	begin
		work   := current_entry;
		work_v := current_valid;
		
		work_boffset := latch_offset * word_t'length;
		
		data_mask := wb_get_data_sel(mem_data_size, mem_address);
		
		if cache_crtl = UPDATE_REFILL then -- replacement update
			work(pol_victim).data  := bus_block_in;
			work(pol_victim).dirty := False;
			work(pol_victim).tag   := latch_tag;
			work_v(pol_victim)     := '1';
		elsif cache_crtl = WRITE_RESPOND then
			for ii in data_mask'range loop
				if data_mask(ii) = '1' then
					work(hit_way).data(work_boffset + (ii + 1) * 8 - 1 downto work_boffset + ii * 8) := proc_word_in((ii + 1) * 8 - 1 downto ii * 8);
				end if;
			end loop;
			work(hit_way).dirty := True;
		end if;
		
		current_entry_update <= work;
		current_valid_update <= work_v;
	end process entry_update;
	
	policy: entity work.pp_dcache_pol_sel
		generic map (
			HAS_LRU    => HAS_LRU,
			HAS_MRU    => HAS_MRU,
			HAS_DLFU   => HAS_DECAYING_LFU,
			HAS_FIFO   => HAS_FIFO,
			HAS_RANDOM => HAS_RANDOM,
			
			DLFU_RATE => DLFU_RATE,
			
			ADAPTIVE_HISTORY => ADAPTIVE_HISTORY,
			
			TAG_BITS    => tag_bits,
			CACHE_DEPTH => CACHE_DEPTH,
			WAYNESS     => WAYNESS
		) port map (
			clk => clk,
			rst => reset,
			
			wb_finished => bus_finished,
			wb_tag      => latch_tag,
			wb_index    => latch_index,
			
			valid => pol_valid_a,
			repl  => pol_victim,
			way   => pol_way,
			index => pol_index,
			tag   => pol_tag,
			tags  => pol_tag_a,
			
			replace  => pol_replace,
			update   => pol_update,
			finished => pol_finished,
			
			crtl => crtl, 
			
			-- trace
			replace_event => replace_event,
			replace_pol   => replace_pol,
			
			-- Random input
			random => random
		);
	pol_index   <= latch_index;
	pol_tag     <= latch_tag;
	pol_way     <= pol_victim when cache_crtl = UPDATE_REFILL else hit_way;
	pol_replace <= '1' when cache_crtl = UPDATE_REFILL else '0';
	pol_update  <= '1' when cache_crtl = READ_RESPOND or cache_crtl = WRITE_RESPOND else '0';
	
	pol_valid_a <= current_valid;
	
	policy_overview: process(current_entry) begin
			for ii in way_t loop
				pol_tag_a((ii + 1) * tag_t'length - 1 downto ii * tag_t'length) <= current_entry(ii).tag;
			end loop;
	end process policy_overview;
	
	bus_finished <=
		'0' when bus_crtl = IDLE else
		'0' when wb_inputs.ack = '0' else
		'1' when bus_start = bus_end else
		'0';
	
end architecture behaviour;

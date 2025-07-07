library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.tracing_types.ALL;

Library xpm;
use xpm.vcomponents.all;

Library UNISIM;
use UNISIM.vcomponents.all;

entity trace_policies is
	generic (
		sample_rate   : integer := 16
	);
	port (
		clk_fr, reset: in std_logic;
		clk_hlt: out std_logic;
		
		enable: in std_logic_vector(1 downto 0);
				
		-- Tracing data
		replace_event : in std_logic;
		replace_pc    : in std_logic_vector(31 downto 0);
		replace_pol   : in policy_t;
		
		-- Wishbone ports:
		wb_cyc_in  : in  std_logic;
		wb_stb_in  : in  std_logic;
		wb_addr_in : in  std_logic_vector(11 downto 0);
		wb_ack_out : out std_logic;
		
		-- Direct dump access to UART buffer
		direct_lock  : out std_logic;
		direct_valid : out std_logic;
		direct_full  : in std_logic;
		direct_data  : out std_logic_vector(7 downto 0)
	);
end trace_policies;

architecture Behavioral of trace_policies is

	signal clk: std_logic;
	
	subtype trace_mem_t is std_logic_vector(53 downto 0);
	signal time_cnt: integer range 0 to 1023;
	
	signal mem_in, mem_out: trace_mem_t;
	
	signal push, empty, pull: std_logic;
	
	type buffer_t is array(0 to 8) of std_logic_vector(5 downto 0);
	signal dump_ctr: buffer_t'range;
	signal dump_buffer_a: buffer_t;
	
	signal count_fifo: std_logic_vector(14 downto 0);
	signal count_in: integer range 0 to 65535;
	
	signal ila_trigger_reg, ila_trigger: std_logic;
	
	signal rd_rst_busy, wr_rst_busy, wr_ack, data_valid: std_logic;
	
	signal count_matches: std_logic;
	
	type ascii_lut_t is array(0 to 63) of std_logic_vector(7 downto 0);
	constant ascii_lut: ascii_lut_t := (
		00 => x"41",
		01 => x"42",
		02 => x"43",
		03 => x"44",
		04 => x"45",
		05 => x"46",
		06 => x"47",
		07 => x"48",
		08 => x"49",
		09 => x"4a",
		10 => x"4b",
		11 => x"4c",
		12 => x"4d",
		13 => x"4e",
		14 => x"4f",
		15 => x"50",
		16 => x"51",
		17 => x"52",
		18 => x"53",
		19 => x"54",
		20 => x"55",
		21 => x"56",
		22 => x"57",
		23 => x"58",
		24 => x"59",
		25 => x"5a",
		26 => x"61",
		27 => x"62",
		28 => x"63",
		29 => x"64",
		30 => x"65",
		31 => x"66",
		32 => x"67",
		33 => x"68",
		34 => x"69",
		35 => x"6a",
		36 => x"6b",
		37 => x"6c",
		38 => x"6d",
		39 => x"6e",
		40 => x"6f",
		41 => x"70",
		42 => x"71",
		43 => x"72",
		44 => x"73",
		45 => x"74",
		46 => x"75",
		47 => x"76",
		48 => x"77",
		49 => x"78",
		50 => x"79",
		51 => x"7a",
		52 => x"30",
		53 => x"31",
		54 => x"32",
		55 => x"33",
		56 => x"34",
		57 => x"35",
		58 => x"36",
		59 => x"37",
		60 => x"38",
		61 => x"39",
		62 => x"2b",
		63 => x"2f"
	);
	
	type state_t is (
		GATHER, GET_LOCK, GET_NEXT, DUMP_PHASE1, DUMP_PHASE2, START_SYMBOL, IDLE_FULL,
		FINALIZE
	);
	signal state: state_t;
	
	signal ct_cnt:  integer range 0 to sample_rate - 1;
	
	signal const_rate_nibble_in, const_rate_nibble_out: std_logic_vector(2 downto 0);
	signal clk_disable, clk_en, push_disable: std_logic;
	signal const_rate_push, const_rate_pull: std_logic;
	
	type ct_state_t is (
		IDLE, AQ_LOCK, BG_SYMBOL, PUSH1, WAIT1, FINISH
	);
	signal ct_state: ct_state_t;
	
		
	signal direct_lock_c, direct_lock_s   : std_logic;
	signal direct_valid_c, direct_valid_s : std_logic;
	signal direct_data_c, direct_data_s   : std_logic_vector(7 downto 0);
	signal ack_c, ack_s : std_logic;
	signal wb_active: std_logic;
	
--	attribute mark_debug : string;
--	attribute mark_debug of direct_lock_c     : signal is "true";
--	attribute mark_debug of direct_valid_c    : signal is "true";
--	attribute mark_debug of direct_lock_s     : signal is "true";
--	attribute mark_debug of direct_valid_s    : signal is "true";
--	attribute mark_debug of clk_en            : signal is "true";
--	attribute mark_debug of ct_state          : signal is "true";
--	attribute mark_debug of state             : signal is "true";
--	attribute mark_debug of wb_active         : signal is "true";
--	attribute mark_debug of push_disable      : signal is "true";
begin
	clk_hlt <= clk;
	clk_en  <=
		'1' when reset = '1' else
		'1' when ct_state /= WAIT1 or ct_state  /= PUSH1 else
		'0' when clk_disable = '1' else
		'1';
	
	wb_active <= wb_stb_in and wb_cyc_in;
	
	BUFGCE_inst : BUFGCE
	port map (
		O => clk,
		CE => clk_en,
		I => clk_fr
	);
	
	direct_lock <=
		direct_lock_c when ct_state /= IDLE else
		direct_lock_s when state /= GATHER  else
		'0';
	direct_valid <=
		direct_valid_c when ct_state /= IDLE else
		direct_valid_s when state /= GATHER  else
		'0';
	direct_data <=
		direct_data_c when ct_state /= IDLE else
		direct_data_s when state /= GATHER  else
		(others => '0');
	wb_ack_out <=
		ack_c when ct_state /= IDLE else
		ack_s when state /= GATHER  else
		'0';
	
	xpm_fifo_async_inst : xpm_fifo_async
		generic map (
			READ_MODE => "std",
			FIFO_READ_LATENCY => 0,
			FULL_RESET_VALUE => 0,
			RD_DATA_COUNT_WIDTH => 1,
			RELATED_CLOCKS => 1,
			FIFO_WRITE_DEPTH => 16,
			READ_DATA_WIDTH => 3,
			WRITE_DATA_WIDTH => 3
		)
		port map (
			rst => reset,
			
			empty => push_disable,
			full => clk_disable,
			
			injectdbiterr => '0',
			injectsbiterr => '0',
			sleep => '0',
			
			rd_clk => clk_fr,
			dout => const_rate_nibble_out,
			rd_en => const_rate_pull,
			
			wr_clk => clk,
			din => const_rate_nibble_in,
			wr_en => const_rate_push
		);

	const_rate_nibble_in <=
		"000" when replace_pol.active = POL_RANDOM else
		"001" when replace_pol.active = POL_FIFO   else
		"010" when replace_pol.active = POL_DLFU   else
		"011" when replace_pol.active = POL_LRU    else
		"100" when replace_pol.active = POL_MRU    else
		"101";
		
	process (clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				ct_cnt <= 0;
			else
				ct_cnt <= (ct_cnt + 1) mod sample_rate;
			end if;
		end if;
	end process;
	
	const_rate_push <=
		'0' when enable(0) = '0' else
		'0' when ct_state = IDLE else
		'0' when ct_state = FINISH else
		'0' when ct_state = BG_SYMBOL else
		'0' when ct_state = AQ_LOCK else
		'0' when wb_cyc_in = '1' and wb_stb_in = '1' else
		'1' when ct_cnt = 0 else
		'0';
	
	const_rate_pull <= '1' when ct_state = PUSH1 else '0';
		
	process (clk_fr) begin
		if rising_edge(clk_fr) then
			if reset = '1' then
				ct_state <= IDLE;
			else
				case ct_state is
					when IDLE =>
						if wb_cyc_in = '1' and wb_stb_in = '1' and wb_addr_in = x"004" and state = GATHER then
							ct_state <= AQ_LOCK;
						end if;
					when AQ_LOCK =>
						if direct_full = '0' then
							ct_state <= BG_SYMBOL;
						end if;
					when BG_SYMBOL => ct_state <= WAIT1;
					when WAIT1 =>
						if direct_full = '0' then
							if wb_cyc_in = '1' and wb_stb_in = '1' and push_disable = '1' then
								ct_state <= FINISH;
							elsif push_disable = '0' then
								ct_state <= PUSH1;
							end if;
						end if;
					when PUSH1 => ct_state <= WAIT1;
					when FINISH => ct_state <= IDLE;
				end case;
			end if;
		end if;
	end process;
	
	direct_lock_c  <= '0' when ct_state = IDLE else '1';
	direct_valid_c <=
		'1' when ct_state = BG_SYMBOL else
		'1' when ct_state = FINISH else
		'1' when ct_state = PUSH1 else
		'0';
	direct_data_c  <=
		x"24" when ct_state = BG_SYMBOL else
		x"24" when ct_state = FINISH else
		x"52" when const_rate_nibble_out = "000" else
		x"46" when const_rate_nibble_out = "001" else
		x"44" when const_rate_nibble_out = "010" else
		x"4C" when const_rate_nibble_out = "011" else
		x"4D" when const_rate_nibble_out = "100" else
		x"4E";
		
	ack_c <=
		'1' when ct_state = FINISH else
		'1' when ct_state = BG_SYMBOL else
		'0';

	st: process (clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				state <= GATHER;
			else
				case state is
					when GATHER =>
						if wb_cyc_in = '1' and wb_stb_in = '1' and wb_addr_in = x"000" and ct_state = IDLE then
							state <= GET_LOCK;
						end if;
					when GET_LOCK =>
						if direct_full = '0' then
							state <= START_SYMBOL;
						end if;
					when START_SYMBOL =>
						state <= DUMP_PHASE2;
						dump_ctr <= 8;
					when FINALIZE => state <= GATHER;
						
					when GET_NEXT =>
						state <= DUMP_PHASE1;
						dump_ctr <= 0;
					when DUMP_PHASE1 => state <= DUMP_PHASE2;
					when DUMP_PHASE2 =>
						if direct_full = '1' then
							state <= IDLE_FULL;
						else
							if empty = '1' then
								state <= FINALIZE;
							elsif dump_ctr = 8 then
								state <= GET_NEXT;
							else
								dump_ctr <= dump_ctr + 1;
								state <= DUMP_PHASE1;
							end if;
						end if;
					when IDLE_FULL =>
						if direct_full = '0' then
							state <= DUMP_PHASE2;
						end if;
				end case;
			end if;
		end if;
	end process st;
	
	direct_lock_s  <= '0' when state = GATHER else '1';
	direct_valid_s <=
		'1' when state = DUMP_PHASE1  else
		'1' when state = FINALIZE     else
		'1' when state = START_SYMBOL else
		'0';
	direct_data_s  <=
		ascii_lut(to_integer(unsigned(dump_buffer_a(dump_ctr)))) when state = DUMP_PHASE1  else
		x"25"                                                    when state = START_SYMBOL else
		x"25"                                                    when state = FINALIZE     else
		(others => '0');
	
	pull  <= '1' when state = GET_NEXT else '0';
	ack_s <= '1' when state = FINALIZE else '0';

	time_counter: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				time_cnt <= 0;
			else
				time_cnt <= (time_cnt + 1) mod 1024;
			end if;
		end if;
	end process time_counter;
	
	push <= replace_event when enable(1) = '1' else '0';
	mem_in <=
		replace_pc(15 downto 2) &
		std_logic_vector(to_unsigned(time_cnt, 10)) &
		std_logic_vector(to_unsigned(replace_pol.random, 6)) &
		std_logic_vector(to_unsigned(replace_pol.fifo, 6)) &
		std_logic_vector(to_unsigned(replace_pol.lru, 6)) &
		std_logic_vector(to_unsigned(replace_pol.mru, 6)) &
		std_logic_vector(to_unsigned(replace_pol.dlfu, 6));

	count_matches <= '1' when count_in = to_integer(unsigned(count_fifo)) else '0';
	instrumentation: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				count_in <= 0;
				ila_trigger_reg <= '1';
			else
				ila_trigger_reg <= wb_cyc_in and wb_stb_in;
				if wb_cyc_in = '1' and wb_stb_in = '1' then
					count_in <= 0;
				elsif replace_event = '1' then
					count_in <= count_in + 1; 
				end if;
			end if;
		end if;
	end process instrumentation;
	
	ila_trigger <= '1' when ila_trigger_reg = '0' and wb_cyc_in = '1' and wb_stb_in = '1' else '0';
	
	decompose: process(mem_out) begin
		for ii in dump_buffer_a'range loop
			dump_buffer_a(ii) <= std_logic_vector(mem_out(6 * (ii + 1) - 1 downto 6 * ii)); 
		end loop;
	end process decompose;
	
--	trace_buffer : entity work.pp_fifo
--	generic map (
--		DEPTH => 32768,
--		WIDTH => 54
--	)
--	port map (
--		clk   => clk,
--		reset => reset,
		
--		empty => empty,
		
--		push => push,
--		pop  => pull,
		
--		data_in  => mem_in,
--		data_out => mem_out
--	);
--	rd_rst_busy <= '0';
--	wr_rst_busy <= '0';
--	wr_ack      <= '0';
--	count_fifo  <= (others => '0');
	
	trace_buffer : xpm_fifo_sync
	generic map (
		FIFO_MEMORY_TYPE    => "block",
		FIFO_WRITE_DEPTH    => 32768,
		RD_DATA_COUNT_WIDTH => 15,
		READ_DATA_WIDTH     => 54,
		READ_MODE           => "std",
		USE_ADV_FEATURES    => "0707",
		WRITE_DATA_WIDTH    => 54,
		WR_DATA_COUNT_WIDTH => 15
	)
	port map (
		wr_clk => clk,
		rst    => reset,
		
		empty => empty,
		
		rd_rst_busy => rd_rst_busy,
		wr_rst_busy => wr_rst_busy,
		wr_ack      => wr_ack,
		
--		rd_data_count => count_fifo,
		wr_data_count => count_fifo,
		
		sleep         => '0',
		injectdbiterr => '0',
		injectsbiterr => '0',
		
		dout       => mem_out,
		data_valid => data_valid,
		rd_en      => pull,
		
		din   => mem_in(53 downto 0),
		wr_en => push
	);
	
--	trace_buffer : entity work.trace_buffer
--	port map (
--		clk => clk,
--		srst => reset,
		
--		data_count => count_fifo,
		
--		dout => mem_out,
--		rd_en => pull,
--		empty => empty,
		
--		din => mem_in,
--		wr_en => push
--	);

end Behavioral;

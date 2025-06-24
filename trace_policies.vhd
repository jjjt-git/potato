library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.tracing_types.ALL;

Library xpm;
use xpm.vcomponents.all;

entity trace_policies is
	port (
		clk, reset: in std_logic;
		
		enable: in std_logic;
				
		-- Tracing data
		replace_event : in std_logic;
		replace_pc    : in std_logic_vector(31 downto 0);
		replace_pol   : in policy_t;
		
		-- Wishbone ports:
		wb_cyc_in  : in  std_logic;
		wb_stb_in  : in  std_logic;
		wb_ack_out : out std_logic;
		
		-- Direct dump access to UART buffer
		direct_lock  : out std_logic;
		direct_valid : out std_logic;
		direct_full  : in std_logic;
		direct_data  : out std_logic_vector(7 downto 0)
	);
end trace_policies;

architecture Behavioral of trace_policies is
	
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
	
	attribute mark_debug : string;
	attribute mark_debug of count_matches : signal is "true";
	attribute mark_debug of empty         : signal is "true";
	attribute mark_debug of push          : signal is "true";
	attribute mark_debug of pull          : signal is "true";
	attribute mark_debug of direct_lock   : signal is "true";
	attribute mark_debug of direct_data   : signal is "true";
	attribute mark_debug of direct_full   : signal is "true";
	attribute mark_debug of direct_valid  : signal is "true";
	attribute mark_debug of rd_rst_busy   : signal is "true";
	attribute mark_debug of wr_rst_busy   : signal is "true";
	attribute mark_debug of wr_ack        : signal is "true";
	attribute mark_debug of data_valid    : signal is "true";
	attribute mark_debug of count_fifo    : signal is "true";
	attribute mark_debug of count_in      : signal is "true";
	
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
		GATHER, GET_LOCK, GET_NEXT, DUMP_PHASE1, DUMP_PHASE2, START_SYMBOL, IDLE_FULL, FINALIZE
	);
	signal state: state_t;
	
	attribute mark_debug of state : signal is "true";
begin

	st: process (clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				state <= GATHER;
			else
				case state is
					when GATHER =>
						if wb_cyc_in = '1' and wb_stb_in = '1' then
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
	
	wb_ack_out <= '1' when state = FINALIZE else '0';
	
	direct_lock  <= '0' when state = GATHER else '1';
	direct_valid <=
		'1' when state = DUMP_PHASE1  else
		'1' when state = FINALIZE     else
		'1' when state = START_SYMBOL else
		'0';
	direct_data  <=
		ascii_lut(to_integer(unsigned(dump_buffer_a(dump_ctr)))) when state = DUMP_PHASE1  else
		x"25"                                                    when state = START_SYMBOL else
		x"25"                                                    when state = FINALIZE     else
		(others => '0');
	
	pull <= '1' when state = GET_NEXT else '0';

	time_counter: process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				time_cnt <= 0;
			else
				time_cnt <= (time_cnt + 1) mod 1024;
			end if;
		end if;
	end process time_counter;
	
	push <= replace_event when enable = '1' else '0';
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

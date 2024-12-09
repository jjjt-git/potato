-- The Potato Processor - A simple processor for FPGAs
-- (c) Kristian Klomsten Skordal 2014 - 2015 <kristian.skordal@wafflemail.net>
-- Report bugs and issues on <https://github.com/skordal/potato/issues>
-- modified to use registered inputs by Jacob Tilger <jacob.tilger@mailbox.tu-dresden.de> 

library ieee;
use ieee.std_logic_1164.all;

use work.pp_types.all;

--! @brief Simple priority-based wishbone arbiter.
--! This module is used as an arbiter between the instruction and data caches.
entity pp_wb_arbiter is
	generic(
		USE_REGISTERED_INPUTS : boolean := false
	);
	port(
		clk   : in std_logic;
		reset : in std_logic;

		-- Wishbone input 1:
		m1_inputs  : out wishbone_master_inputs;
		m1_outputs : in  wishbone_master_outputs;

		-- Wishbone input 2:
		m2_inputs  : out wishbone_master_inputs;
		m2_outputs : in  wishbone_master_outputs;

		-- Wishbone interface:
		wb_adr_out : out std_logic_vector(31 downto 0);
		wb_sel_out : out std_logic_vector( 3 downto 0);
		wb_cyc_out : out std_logic;
		wb_stb_out : out std_logic;
		wb_we_out  : out std_logic;
		wb_dat_out : out std_logic_vector(31 downto 0);
		wb_dat_in  : in  std_logic_vector(31 downto 0);
		wb_ack_in  : in  std_logic
	);
end entity pp_wb_arbiter;

architecture behaviour of pp_wb_arbiter is

	type state_type is (IDLE, M1_BUSY, M2_BUSY);
	signal state : state_type := IDLE;
	
	signal m1_outputs_buffer, m2_outputs_buffer : wishbone_master_outputs;
	signal wb_dat_in_buffer : std_logic_vector(31 downto 0);
	signal wb_ack_in_buffer : std_logic;

begin

	m1_inputs <= (ack => wb_ack_in_buffer, dat => wb_dat_in_buffer) when state = M1_BUSY else (ack => '0', dat => (others => '0'));
	m2_inputs <= (ack => wb_ack_in_buffer, dat => wb_dat_in_buffer) when state = M2_BUSY else (ack => '0', dat => (others => '0'));
	
	registered_inputs: if USE_REGISTERED_INPUTS generate
		process(clk) begin
			if rising_edge(clk) then
				m1_outputs_buffer <= m1_outputs;
				m2_outputs_buffer <= m2_outputs;
				wb_dat_in_buffer <= wb_dat_in;
				wb_ack_in_buffer <= wb_ack_in;
			end if;
		end process;
	end generate registered_inputs;
	
	unregistered_inputs: if not USE_REGISTERED_INPUTS generate
		m1_outputs_buffer <= m1_outputs;
		m2_outputs_buffer <= m2_outputs;
		wb_dat_in_buffer <= wb_dat_in;
		wb_ack_in_buffer <= wb_ack_in;
	end generate unregistered_inputs;

	output_mux: process(state, m1_outputs_buffer, m2_outputs_buffer)
	begin
		case state is
			when IDLE =>
				wb_adr_out <= (others => '0');
				wb_sel_out <= (others => '0');
				wb_dat_out <= (others => '0');
				wb_cyc_out <= '0';
				wb_stb_out <= '0';
				wb_we_out <= '0';
			when M1_BUSY =>
				wb_adr_out <= m1_outputs_buffer.adr;
				wb_sel_out <= m1_outputs_buffer.sel;
				wb_dat_out <= m1_outputs_buffer.dat;
				wb_cyc_out <= m1_outputs_buffer.cyc;
				wb_stb_out <= m1_outputs_buffer.stb;
				wb_we_out <= m1_outputs_buffer.we;
			when M2_BUSY =>
				wb_adr_out <= m2_outputs_buffer.adr;
				wb_sel_out <= m2_outputs_buffer.sel;
				wb_dat_out <= m2_outputs_buffer.dat;
				wb_cyc_out <= m2_outputs_buffer.cyc;
				wb_stb_out <= m2_outputs_buffer.stb;
				wb_we_out <= m2_outputs_buffer.we;
		end case;
	end process output_mux;

	controller: process(clk)
	begin
		if rising_edge(clk) then
			if reset = '1' then
				state <= IDLE;
			else
				case state is
					when IDLE =>
						if m1_outputs_buffer.cyc = '1' then
							state <= M1_BUSY;
						elsif m2_outputs_buffer.cyc = '1' then
							state <= M2_BUSY;
						end if;
					when M1_BUSY =>
						if m1_outputs_buffer.cyc = '0' then
							state <= IDLE;
						end if;
					when M2_BUSY =>
						if m2_outputs_buffer.cyc = '0' then
							state <= IDLE;
						end if;
				end case;
			end if;
		end if;
	end process controller;

end architecture behaviour;

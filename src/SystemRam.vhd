----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 02/10/2025 11:15:35 AM
-- Design Name: 
-- Module Name: pp_soc_memory - BehavioralInit
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.math_real.all;

Library xpm;
use xpm.vcomponents.all;

use work.pp_utilities.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity pp_soc_memory is
	generic(
		MEMORY_SIZE : natural := 4096; --! Memory size in bytes.
		MEMORY_INIT_FILE : string := "none";
		MEMORY_LATENCY : natural := 2
	);
	port(
		clk : in std_logic;
		reset : in std_logic;

		-- Wishbone interface:
		wb_adr_in  : in  std_logic_vector(log2(MEMORY_SIZE) - 1 downto 0);
		wb_dat_in  : in  std_logic_vector(31 downto 0);
		wb_dat_out : out std_logic_vector(31 downto 0);
		wb_cyc_in  : in  std_logic;
		wb_stb_in  : in  std_logic;
		wb_sel_in  : in  std_logic_vector( 3 downto 0);
		wb_we_in   : in  std_logic;
		wb_ack_out : out std_logic
	);
end entity pp_soc_memory;

architecture BehavioralInit of pp_soc_memory is
	subtype latency_t is integer range 0 to MEMORY_LATENCY - 1;
	
	signal latency : latency_t;

	signal read_data : std_logic_vector(31 downto 0);
	signal data_mask : std_logic_vector(31 downto 0);
	
	signal write_mask : std_logic_vector(3 downto 0);
	signal enable, enable_out, enable_in : std_logic;

begin

	-- xpm_memory_spram: Single Port RAM
	-- Xilinx Parameterized Macro, version 2024.2
	
	ram : xpm_memory_spram
	generic map (
		ADDR_WIDTH_A => log2(MEMORY_SIZE) - 2,
		AUTO_SLEEP_TIME => 0,
		BYTE_WRITE_WIDTH_A => 8,
		CASCADE_HEIGHT => 0,
		ECC_BIT_RANGE => "7:0",
		ECC_MODE => "no_ecc",
		ECC_TYPE => "none",
		IGNORE_INIT_SYNTH => 0,
		MEMORY_INIT_FILE => MEMORY_INIT_FILE,
		MEMORY_INIT_PARAM => "0",
		MEMORY_OPTIMIZATION => "true",
		MEMORY_PRIMITIVE => "block",
		MEMORY_SIZE => MEMORY_SIZE * 8,
		MESSAGE_CONTROL => 1,
		RAM_DECOMP => "power",
		READ_DATA_WIDTH_A => 32,
		READ_LATENCY_A => latency_t'high - 1,
		READ_RESET_VALUE_A => "0",
		RST_MODE_A => "SYNC",
		SIM_ASSERT_CHK => 1,
		USE_MEM_INIT => 1,
		USE_MEM_INIT_MMI => 1,
		WAKEUP_TIME => "disable_sleep",
		WRITE_DATA_WIDTH_A => 32,
		WRITE_MODE_A => "read_first",
		WRITE_PROTECT => 1
	)
	port map (
		clka => clk,
		rsta => reset,
		
		ena   => enable_in,
		addra => wb_adr_in(log2(MEMORY_SIZE) - 1 downto 2),
		douta => read_data,
		wea => write_mask,
		dina  => wb_dat_in,
		
		injectdbiterra => '0',
		injectsbiterra => '0',
		regcea         => enable_out,
		sleep          => '0'
	);

	data_mask <=
		(others => '0') when enable = '0' else
		(others => '0') when wb_we_in = '1' else 
		(
			31 downto 24 => wb_sel_in(3),
			23 downto 16 => wb_sel_in(2),
			15 downto  8 => wb_sel_in(1),
			 7 downto  0 => wb_sel_in(0)
		);
		
	write_mask <=
		(others => '0') when enable = '0' else
		(others => '0') when wb_we_in = '0' else
		wb_sel_in;
	
	enable <= wb_cyc_in and wb_stb_in;

	wb_dat_out <= read_data and data_mask;

	wb_ack_out <=
		'0' when enable = '0' else
		'1' when latency = latency_t'high else
		'0';
	
	enable_out <=
		'0' when enable  = '0' else
		'1' when latency = latency_t'high - 1 else
		'0';
	
	enable_in <=
		'0' when enable  = '0' else
		'1' when latency = 0 else
		'0';

	wishbone: process(clk)
	begin
		if rising_edge(clk) then
			if reset = '1' then
				latency <= 0;
			else
				if enable = '1' and latency /= latency_t'high then
					latency <= latency + 1;
				else
					latency <= 0;
				end if;
			end if;
		end if;
	end process wishbone;

end BehavioralInit;
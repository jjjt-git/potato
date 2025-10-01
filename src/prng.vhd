----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05/28/2025 03:08:14 PM
-- Design Name: 
-- Module Name: prng - Behavioral
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
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity prng is
	generic (
		WOUT  : integer := 32;
		W0    : integer := 18;
		W1    : integer := 25;
		init0 : integer := 1;
		init1 : integer := 1
	);
	port (
		clk    : in std_logic;
		reset  : in std_logic;
		random : out std_logic_vector(WOUT - 1 downto 0)
	);
end prng;

architecture Behavioral of prng is
	signal di0: std_logic_vector(W0 - 1 downto 0); 
	signal do0: std_logic_vector(W0 - 1 downto 0);
	signal di1: std_logic_vector(W1 - 1 downto 0);
	signal do1: std_logic_vector(W1 - 1 downto 0);
	
	signal prod: signed(W0 + W1 - 1 downto 0);
	signal acc:  signed(W0 + W1 - 1 downto 0);
begin

	shift0: entity work.polynomial_fb_shift
		generic map(
			WIDTH => W0
		) port map(
			din  => di0,
			dout => do0
		);
	shift1: entity work.polynomial_fb_shift
		generic map(
			WIDTH => W1
		) port map(
			din  => di1,
			dout => do1
		);
		
	prod <= signed(di0) * signed(di1);
	
	process(clk) begin
		if rising_edge(clk) then
			if reset = '1' then
				acc <= (others => '0');
				di0 <= std_logic_vector(to_unsigned(init0, W0));
				di1 <= std_logic_vector(to_unsigned(init1, W1));
			else
				acc <= prod;
				di0 <= do0;
				di1 <= do1;
			end if;
		end if;
	end process;
		
	random <= std_logic_vector(acc(WOUT - 1 downto 0));

end Behavioral;

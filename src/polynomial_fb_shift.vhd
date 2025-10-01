----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05/28/2025 03:30:29 PM
-- Design Name: 
-- Module Name: polynomial_fb_shift - Behavioral
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity polynomial_fb_shift is
	generic (
		WIDTH : integer := 16
	);
	port (
		din : in std_logic_vector(WIDTH - 1 downto 0);
		dout : out std_logic_vector(WIDTH - 1 downto 0)
	);
end polynomial_fb_shift;

architecture XAPP052_coefficients of polynomial_fb_shift is
	signal xor_vec : std_logic_vector(WIDTH - 1 downto 0);

	type coeff_t is array (3 to 64) of std_logic_vector(63 downto 0);
	constant coefficients : coeff_t := (
		x"0000000000000006", -- 3
		x"000000000000000c", -- 4
		x"0000000000000014", -- 5
		x"0000000000000030", -- 6
		x"0000000000000060", -- 7
		x"00000000000000b8", -- 8
		x"0000000000000110", -- 9
		x"0000000000000240", -- 10
		x"0000000000000500", -- 11
		x"0000000000000829", -- 12
		x"000000000000100d", -- 13
		x"0000000000002015", -- 14
		x"0000000000006000", -- 15
		x"000000000000d008", -- 16
		x"0000000000012000", -- 17
		x"0000000000020400", -- 18
		x"0000000000040023", -- 19
		x"0000000000090000", -- 20
		x"0000000000140000", -- 21
		x"0000000000300000", -- 22
		x"0000000000420000", -- 23
		x"0000000000e10000", -- 24
		x"0000000001200000", -- 25
		x"0000000002000023", -- 26
		x"0000000004000013", -- 27
		x"0000000009000000", -- 28
		x"0000000014000000", -- 29
		x"0000000020000029", -- 30
		x"0000000048000000", -- 31
		x"0000000080200003", -- 32
		x"0000000100080000", -- 33
		x"0000000204000003", -- 34
		x"0000000500000000", -- 35
		x"0000000801000000", -- 36
		x"000000100000001f", -- 37
		x"0000002000000031", -- 38
		x"0000004400000000", -- 39
		x"000000a000140000", -- 40
		x"0000012000000000", -- 41
		x"00000300000c0000", -- 42
		x"0000063000000000", -- 43
		x"00000c0000030000", -- 44
		x"00001b0000000000", -- 45
		x"0000300003000000", -- 46
		x"0000420000000000", -- 47
		x"0000c00000180000", -- 48
		x"0001008000000000", -- 49
		x"0003000000c00000", -- 50
		x"0006000c00000000", -- 51
		x"0009000000000000", -- 52
		x"0018003000000000", -- 53
		x"0030000000030000", -- 54
		x"0040000040000000", -- 55
		x"00c0000600000000", -- 56
		x"0102000000000000", -- 57
		x"0200004000000000", -- 58
		x"0600003000000000", -- 59
		x"0c00000000000000", -- 60
		x"1800300000000000", -- 61
		x"3000000000000030", -- 62
		x"6000000000000000", -- 63
		x"d800000000000000"  -- 64
	);
begin
	xor_vec <= coefficients(WIDTH)(WIDTH - 1 downto 0) when din(0) = '1' else (others => '0');
	dout    <= std_logic_vector(shift_right(unsigned(din), 1)) xor xor_vec; 
end XAPP052_coefficients;

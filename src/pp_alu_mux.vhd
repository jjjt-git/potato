-- The Potato Processor - A simple processor for FPGAs
-- (c) Kristian Klomsten Skordal 2014 <kristian.skordal@wafflemail.net>
-- Report bugs and issues on <https://github.com/skordal/potato/issues>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.pp_types.all;

--! @brief Multiplexer used to choose between ALU inputs.
entity pp_alu_mux is
	port(
		source : in alu_operand_source;

		register_value  : in std_logic_vector(31 downto 0);
		immediate_value : in std_logic_vector(31 downto 0);
		shamt_value     : in std_logic_vector( 4 downto 0);
		pc_value        : in std_logic_vector(31 downto 0);
		csr_value       : in std_logic_vector(31 downto 0);

		output_value : out std_logic_vector(31 downto 0)
	);
end entity pp_alu_mux;

architecture behaviour of pp_alu_mux is
begin

	mux: process(source, register_value, immediate_value, shamt_value, pc_value, csr_value)
	begin
		case source is
			when ALU_SRC_REG =>
				output_value <= register_value;
			when ALU_SRC_IMM =>
				output_value <= immediate_value;
			when ALU_SRC_PC =>
				output_value <= pc_value;
			when ALU_SRC_PC_NEXT =>
				output_value <= std_logic_vector(unsigned(pc_value) + 4);
			when ALU_SRC_CSR =>
				output_value <= csr_value;
			when ALU_SRC_SHAMT =>
				output_value <= (31 downto 5 => '0') & shamt_value;
			when ALU_SRC_NULL =>
				output_value <= (others => '0');
		end case;
	end process mux;

end architecture behaviour;

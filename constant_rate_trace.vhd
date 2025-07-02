----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/02/2025 12:22:25 PM
-- Design Name: 
-- Module Name: constant_rate_trace - Behavioral
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

use work.tracing_types.ALL;

entity constant_rate_trace is
	generic (
		sample_rate : integer := 16;
	
		ETH_src_ip,   ETH_dst_ip   : std_logic_vector(31 downto 0);
		ETH_src_mac,  ETH_dst_mac  : std_logic_vector(47 downto 0);
		ETH_src_port, ETH_dst_port : integer range 0 to 65535
	);
	port (
		sample_clk, eth_mac_clk, eth_phy_clk : in std_logic;
		reset, reset_eth : in std_logic;
		eth_ready : out std_logic;
		eth_flush : in std_logic;
		
		enable      : in std_logic;
		replace_pol : in policy_t;
		
		RMII_rclk             : out std_logic;
		RMII_crsdv, RMII_rxer : in  std_logic;
		RMII_rxd              : in  std_logic_vector(1 downto 0);
		RMII_txen             : out std_logic;
		RMII_txd              : out std_logic_vector(1 downto 0)
	);
end constant_rate_trace;

architecture Behavioral of constant_rate_trace is
	signal MII_rclk, MII_rstn, MII_txclk, MII_txen : std_logic;
	signal MII_txd : std_logic_vector(3 downto 0);
	
	subtype pkt_ctr_t  is integer range 0 to 65535;
	subtype word_ctr_t is integer range 0 to 273;
	subtype rate_ctr_t is integer range 0 to sample_rate - 1;
	subtype dump_ctr_t is integer range 0 to 7;
	type    dump_arr_t is array(dump_ctr_t) of std_logic_vector(3 downto 0);
	
	signal sample_wr, sample_rd, dump_en : std_logic;
	signal sample_buf, dump_buf : std_logic_vector(31 downto 0);
	signal dump_buf_a: dump_arr_t;
	
	signal sample_ctr : rate_ctr_t;
	signal dump_ctr   : dump_ctr_t;
	signal packet_num : pkt_ctr_t;
	signal packet_pos : word_ctr_t;
	
	signal endnote    : std_logic;
	
	signal dump_nibble : std_logic_vector(3 downto 0);
begin

	dump_nibble <=
		dump_buf_a(dump_ctr + 1) when dump_ctr mod 2 = 0 else
		dump_buf_a(dump_ctr - 1);
	
	decomp: process(dump_buf) begin
		for ii in dump_buf_a'range loop
			dump_buf_a(ii) <= dump_buf((ii + 1) * 4 - 1 downto ii * 4);
		end loop;
	end process decomp;
	
	dump_buf <=
		std_logic_vector(to_unsigned(packet_num, 32)) when packet_pos = 0 else
		sample_buf;
	
	dump_en <=
		'1' when packet_pos = 0 else
		'0' when sample_wr = sample_rd else
		'1';
		
	process(sample_clk) begin
		if rising_edge(sample_clk) then
			if reset = '1' then
				sample_wr  <= '0';
				sample_ctr <= 0;
			else
				if enable = '1' then
					endnote <= '0';
					sample_ctr <= (sample_ctr + 1) mod (rate_ctr_t'high + 1);
					if sample_ctr = 0 then
						sample_wr <= not sample_wr;
						sample_buf <=
							"00" &
							std_logic_vector(to_unsigned(replace_pol.random, 6)) &
							std_logic_vector(to_unsigned(replace_pol.fifo, 6)) &
							std_logic_vector(to_unsigned(replace_pol.lru, 6)) &
							std_logic_vector(to_unsigned(replace_pol.mru, 6)) &
							std_logic_vector(to_unsigned(replace_pol.dlfu, 6));
					end if;
				else
					if endnote = '0' and sample_wr = sample_rd then
						endnote <= '1';
						sample_wr <= not sample_wr;
						sample_buf <= (others => '1');
					end if;
				end if;
			end if;
		end if;
	end process;
		
	process(eth_mac_clk) begin
		if rising_edge(eth_mac_clk) then
			if reset = '1' then
				dump_ctr   <= 4;
				packet_num <= 0;
				packet_pos <= 0;
				sample_rd  <= '0';
			else
				if packet_pos = 0 then -- dump counter
					dump_ctr <= (dump_ctr + 1) mod (dump_ctr_t'high + 1);
					if dump_ctr = dump_ctr_t'high then
						packet_pos <= 1;
					end if;
				else
					if dump_ctr = dump_ctr_t'high then
						sample_rd <= not sample_rd;
					end if;
					if packet_pos = word_ctr_t'high then
						if dump_ctr = dump_ctr_t'high then
							dump_ctr <= 4;
							packet_pos <= 0;
							packet_num <= (packet_num + 1) mod (pkt_ctr_t'high + 1);
						else
							dump_ctr <= dump_ctr + 1;
						end if;
					else
						dump_ctr <= (dump_ctr + 1) mod (dump_ctr_t'high + 1);
						if dump_ctr = dump_ctr_t'high then
							packet_pos <= packet_pos + 1;
						end if;
					end if;
				end if;
			end if;
		end if;
	end process;

	udp_streamer: entity work.eth_udp_send_wrapper
	generic map (
		CLK_RATIO => 1,
		WORD_SIZE_BYTES => 2,
		MAX_DATA_BYTES => 546,
		MIN_DATA_BYTES => 546
	) port map (
		clk   => eth_mac_clk,
		clk25 => eth_mac_clk,
		rst   => reset_eth,
		
		rdy => eth_ready,
		
		wr_en => dump_en,
		wr_d  => dump_nibble,
		
		flush => eth_flush,
		
		IEthPhy_ref_clk => MII_rclk,
		IEthPhy_rstn    => MII_rstn,
		IEthPhy_tx_clk  => MII_txclk,
		IEthPhy_tx_en   => MII_txen,
		IEthPhy_tx_d    => MII_txd,
		
		IIpInfo_src_ip   => ETH_src_ip,
		IIpInfo_src_mac  => ETH_src_mac,
		IIpInfo_src_port => std_logic_vector(to_unsigned(ETH_src_port, 16)),
		IIpInfo_dst_ip   => ETH_dst_ip,
		IIpInfo_dst_mac  => ETH_dst_mac,
		IIpInfo_dst_port => std_logic_vector(to_unsigned(ETH_dst_port, 16))
	);

	RMII_rclk <= eth_phy_clk;
	mii_to_rmii: entity work.rmii_phy_if
	port map (
		mode_speed => '1',
		
		rstn_async => MII_rstn,
		
		mac_mii_txd  => MII_txd,
		mac_mii_txc  => MII_txclk,
		mac_mii_txen => MII_txen,
		mac_mii_txer => '0',
		
		phy_rmii_ref_clk => eth_phy_clk,
		phy_rmii_crsdv   => RMII_crsdv,
		phy_rmii_rxer    => RMII_rxer,
		phy_rmii_rxd     => RMII_rxd,
		phy_rmii_txen    => RMII_txen,
		phy_rmii_txd     => RMII_txd
	);
	
end Behavioral;

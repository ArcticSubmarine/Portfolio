library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library source;
use source.all;
library AESLibrary;
use AESLibrary.all;
use AESLibrary.state_definition_package.all;

-- Focntionnel

entity InvShiftRows_tb is
end entity InvShiftRows_tb;

architecture InvShiftRows_tb_arch of InvShiftRows_tb is
	component InvShiftRows
		port (	InvShiftRows_i : in type_state;
			InvShiftRows_o : out type_state);
	end component;

	signal e1_s : type_state;	--Signal d'entrée de InvShiftROws
	signal s1_s : type_state;	--Signal de sortie de InvShiftRows

	begin
		DUT : InvShiftRows
		port map (		InvShiftRows_i => e1_s,
				        InvShiftRows_o => s1_s);

	P0 : process
		begin
			e1_s <= (	(x"06", x"fb", x"5f", x"74"),
					(x"85", x"06", x"ca", x"5b"),
					(x"a6", x"54", x"99", x"8e"),
					(x"61", x"09", x"c1", x"56"));

			wait for 10 ns;

	end process;
end architecture;

-- Sortie attendue : 06 fb 5f 74 5b 85 06 ca 99 8e A6 54 09 C1 56 61
-- vcom -work source ./bench/InvShiftRows_tb.vhd
-- vsim bench InvShiftRows_tb.vhd
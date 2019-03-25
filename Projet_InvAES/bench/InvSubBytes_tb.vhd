library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library source;
use source.all;
library AESLibrary;
use AESLibrary.all;
use AESLibrary.state_definition_package.all;

--Fonctionne

entity InvSubBytes_tb is
end entity;

architecture InvSubBytes_tb_arch of InvSubBytes_tb is
	component InvSubBytes
		port (	InvSubBytes_i : in type_state;
			InvSubBytes_o : out type_state); 
	end component;
	
	signal e1_s : type_state;	--Signal d'entrée de InvSubBytes
	signal s1_s : type_state;	--Signal de sortie

	begin
		DUT : InvSubBytes
		port map (	InvSubBytes_i => e1_s,
				InvSubBytes_o => s1_s);

	P0 : process
		begin
			e1_s <= (	(x"06", x"09", x"99", x"5b"), 
					(x"85", x"fb", x"c1", x"8e"), 
					(x"a6", x"06", x"5f", x"56"), 
					(x"61", x"54", x"ca", x"74"));
	                
			wait for 10 ns;

	end process;
end architecture;

-- Sortie attendue : a5 40 f9 57 67 63 dd e6 c5 a5 84 b9 d8 fd 10 ca

-- vcom -work source ./bench/InvSubBytes_tb.vhd
-- vsim bench InvSubBytes_tb.vhd

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

--Fonctionnel

entity InvMixColumns_tb is
end entity;

architecture InvMixColumns_tb_arch of InvMixColumns_tb is
	component InvMixColumns
		port (	InvMixColumns_i : in  type_state;	
			InvMixColumns_o : out type_state); 
	end component;

	signal e1_s : type_state;
	signal s1_s : type_state;

	begin
		DUT : InvMixColumns
		port map (	InvMixColumns_i => e1_s,
				InvMixColumns_o => s1_s);

	P0 : process
		begin 
			e1_s <= (	(x"ce", x"69", x"c8", x"5c"),
					(x"3f", x"86", x"41", x"46"),
					(x"7d", x"66", x"97", x"7f"),
					(x"8b", x"77", x"4d", x"11"));
			wait for 10 ns;
	end process;

end architecture;

-- Sortie attendue : 33 54 53 07 8c 07 95 a0 e4 d1 f3 35 8a 57 17 6a

-- vcom -work source ./bench/InvMixColumns_tb.vhd
-- vsim bench InvMixColumns_tb.vhd

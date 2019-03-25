library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

--Fonctionnel

entity AddRoundKey_tb is
end entity;


architecture AddRoundKey_tb_arch of AddRoundKey_tb is
	component AddRoundKey
		port (	AddRoundKey_i 	: in type_state;
			Expansion_key_i : in type_state;
			AddRoundKey_o 	: out type_state); 
	end component;
	
	signal e1_s : type_state;	--Signal d'entrée
	signal k1_s : type_state;	--Clé courante
	signal s1_s : type_state;	--Signal de sortie

	begin
		DUT : AddRoundKey
		port map (	AddRoundKey_i 	=> e1_s,
				Expansion_key_i => k1_s,
				AddRoundKey_o 	=> s1_s);

	P0 : process
		begin
			e1_s <= (	(x"04", x"e0", x"48", x"28"), 
					(x"66", x"cb", x"f8", x"06"), 
					(x"81", x"19", x"d3", x"26"), 
					(x"e5", x"9a", x"7a", x"4c"));

	 --Exemple du poly figure 8

		
	                k1_s <= (	(x"a0", x"88", x"23", x"2a"), 
					(x"fa", x"54", x"a3", x"6c"), 
					(x"fe", x"2c", x"39", x"76"), 
					(x"17", x"b1", x"39", x"05"));

	

			wait for 10 ns;

	end process;
end architecture;

-- Résultat attendu : a4 68 6b 02 9c 9f 5b 6a 7f 35 ea 50 f2 2b 43 49

-- vcom -work source ./bench/AddRoundKey_tb.vhd
-- vsim bench AddRoundKey_tb.vhd

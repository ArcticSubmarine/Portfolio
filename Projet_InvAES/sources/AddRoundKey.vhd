library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;



entity AddRoundKey is
	port (	AddRoundKey_i 	: in type_state;	--Texte courant	
		Expansion_key_i : in type_state;	--Clé courante 
		AddRoundKey_o 	: out type_state);	--Texte "xoré" avec la clé courante
end entity AddRoundKey;

architecture AddRoundKey_arch of AddRoundKey is
	begin
		label1 : for i in 0 to 3 generate
			label2 : for j in 0 to 3 generate
				AddRoundKey_o(i)(j) <= AddRoundKey_i(i)(j) xor (Expansion_key_i(i)(j));		--Opération xor entre le texte d'entrée et la clé courante.
			end generate;
		end generate;
	
end architecture AddRoundKey_arch;

-- vcom -work source ./sources/AddRoundKey.vhd

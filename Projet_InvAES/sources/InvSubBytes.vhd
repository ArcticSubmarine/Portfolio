library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;


entity InvSubBytes is 
port (	InvSubBytes_i : in type_state;
	InvSubBytes_o : out type_state);
end entity InvSubBytes;


architecture InvSubBytes_arch of InvSubBytes is
	component InvSBOX
		port(	InvSBOX_i : in bit8;
			InvSBOX_o : out bit8);
	end component InvSBOX;

	begin						--Sélection des données correspondantes au signal d'entrée dans la matrice de InvSBOX.
		label1 : for i in 0 to 3 generate
			label2 : for j in 0 to 3 generate
				u : InvSBOX port map (
					InvSBOX_i => InvSubBytes_i(i)(j),
					InvSBOX_o => InvSubBytes_o(i)(j));
			end generate;
		end generate;
end architecture InvSubBytes_arch;

				
-- vcom -work source ./sources/InvSubBytes.vhd


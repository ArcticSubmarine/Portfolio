library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

entity InvMixColumns is 
		port (	InvMixColumns_i : in type_state;
			InvMixColumns_o : out type_state);
end entity InvMixColumns;


architecture InvMixColumns_arch of InvMixColumns is
	component InvMixColumns_ROW is
		port (	InvMixColumns_ROW_i : in row_state;
			InvMixColumns_ROW_o : out row_state);
	end component InvMixColumns_ROW;
	
	signal InvMixColumns_i_s	: type_state;
	signal InvMixColumns_o_s	: type_state;

	begin	
		--Effectue les transformations nécessaire sur chacune des lignes du type_state donné en entrée en utilisant InvMixColumns_ROW, qui effectue celles-ci ligne par ligne.
		G1 : for i in 0 to 3 generate
			u : InvMixColumns_ROW
			port map (	InvMixColumns_ROW_i	=> InvMixColumns_i(i),
					InvMixColumns_ROW_o	=> InvMixColumns_o(i));
		end generate G1;

end architecture InvMixColumns_arch;

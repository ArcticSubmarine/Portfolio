library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

entity InvAESRound_tb is
end entity InvAESRound_tb;

architecture InvAESRound_tb_arch of InvAESRound_tb is
	component InvAESRound is 
		port (	clock_i			: in std_logic;	--Signal d'horloge
			currentKey_i		: in bit128;	--Clé courrante
			currentText_i		: in bit128;	--Texte courrant
			enableInvMixColumns_i	: in std_logic;	--Signal autorisant InvMixColumns
			enableRoundComputing_i	: in std_logic;	--Signal autorisant RoundComputing
			resetb_i		: in std_logic;	--Signal de reset
			data_o			: out bit128);	--Sortie
	end component InvAESRound;

	signal clock_s			: std_logic;
	signal currentKey_s		: bit128;
	signal currentText_s		: bit128;
	signal enableInvMixColumns_s	: std_logic;
	signal enableRoundComputing_s	: std_logic;
	signal resetb_s			: std_logic;
	signal data_s			: bit128;

	begin
		DUT : InvAESRound
		port map (	clock_i			=> clock_s,
				currentKey_i		=> currentKey_s,
				currentText_i		=> currentText_s,
				enableInvMixColumns_i	=> enableInvMixColumns_s,
				enableRoundComputing_i	=> enableRoundComputing_s,
				resetb_i		=> resetb_s
				data_o			=> data_s);

		
		
		P0 : process		--Simulation d'un signal d'horloge
		begin
			clock_s <= '0';
			wait for 15 ns;
			clock_s <= '1';
			wait for 15 ns;
		end process P0;

		P1 : process 
		begin
			currentKey_s 	<= (x"a088232afa54a36cfe2c397617b13905");
			wait for 10 ns;
			currentText_s 	<= (x"04e0482866cbf8068119d326e59a7a4c");
			wait for 10 ns;
		end process P1;			

		P2 : process
		begin
			resetb_s		<= '0';
			wait for 30 ns;
			enableRoundComputing_s	<= '1';
			wait for 10 ns;
			enableInvMixColumns_s	<= '1';
			wait for 10 ns;
			enableRoundComputing_s	<= '0';
			wait;
		end process P2;		

end architecture InvAESRound_tb_arch;


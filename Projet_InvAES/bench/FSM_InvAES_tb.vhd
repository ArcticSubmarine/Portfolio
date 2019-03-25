Library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

--Fonctionne

entity FSM_InvAES_Moore_tb is 
end entity FSM_InvAES_Moore_tb;

architecture FSM_InvAES_Moore_tb_arch of FSM_InvAES_Moore_tb is
	component FSM_InvAES_Moore is 
		port (clock_i			:	in std_logic;
		      resetb_i			: 	in std_logic;
		      start_i			: 	in std_logic;
		      round_i			: 	in bit4;
		      done_o			: 	out std_logic;
		      enableCounter_o 		:	out std_logic;
		      enableMixColumns_o	: 	out std_logic;
		      enableOutput_o		:  	out std_logic;
		      enableRoundComputing_o 	: 	out std_logic;
		      getCipherText_o		: 	out std_logic;
		      resetCounter_o		:	out std_logic);
	end component FSM_InvAES_Moore;

	component Counter is 
		port (	clock_i		:	in std_logic;		-- entrée horloge
			enable_i	:	in std_logic;		-- signal enable compteur
			resetb_i	:	in std_logic;		-- signal de reset
			count_o		:	out bit4);		-- sortie du compteur
	end component Counter;

	signal clock_s			:	std_logic;
	signal resetb_s			: 	std_logic;
      	signal start_s			: 	std_logic;
      	signal round_s			: 	bit4;
      	signal done_s			: 	std_logic;
      	signal enableCounter_s 		:	std_logic;
      	signal enableMixColumns_s	: 	std_logic;
      	signal enableOutput_s		:  	std_logic;
      	signal enableRoundComputing_s 	: 	std_logic;
      	signal getCipherText_s		: 	std_logic;
      	signal resetCounter_s		:	std_logic;

	
	begin

	DUT : FSM_InvAES_Moore
	port map (	clock_i			=> clock_s,
			resetb_i 		=> resetb_s,
			start_i			=> start_s,
			round_i			=> round_s,
			done_o			=> done_s,
			enableCounter_o		=> enableCounter_s,
			enableMixColumns_o	=> enableMixColumns_s,
			enableOutput_o		=> enableOutput_s,
			enableRoundComputing_o	=> enableRoundComputing_s,
			getCipherText_o		=> getCipherText_s,
			resetCounter_o		=> resetCounter_s);


	P0 : process		--Simulation d'un signal d'horloge
	begin
		clock_s <= '0';
		wait for 15 ns;
		clock_s <= '1';
		wait for 15 ns;
		end process;

	P1 : process		--Reset puis start
	begin
		resetb_s <= '1';
		wait for 45 ns;
		resetb_s <= '0';
		start_s <= '1';
		wait;
	end process P1;

	P2 : process
	begin
		wait for 50 ns;
		round_s	<= "0001";
		wait for 100 ns;
		round_s <= "1001";
		wait;
	end process P2;

end architecture FSM_InvAES_Moore_tb_arch;


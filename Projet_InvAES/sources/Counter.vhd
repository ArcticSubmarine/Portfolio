library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

--FONCTIONNEL

entity Counter is 
	port (	clock_i		:	in std_logic;		-- entrée horloge
		enable_i	:	in std_logic;		-- signal enable compteur
		resetb_i	:	in std_logic;		-- signal de reset
		count_o		:	out bit4);		-- sortie du compteur
end Counter;


architecture Counter_arch of Counter is
	signal counter_s : bit4;

	begin 
		P0 : process (clock_i, resetb_i)
		begin
			if resetb_i = '1' then counter_s <= "0000";	-- remise à 0 si le signal de reset vaut 1

			elsif (clock_i = '1') then 			-- incrémentation si l'horloge est à 1 et si le compteur est enable
				if enable_i = '0' then 
					if counter_s = "1111" then 
						counter_s <= "0000";
					else
						counter_s <= counter_s + '1';
					end if;
				end if;
			end if;
		end process;
		count_o <= counter_s;
end Counter_arch;

--vcom -work source ./sources/Counter.vhd

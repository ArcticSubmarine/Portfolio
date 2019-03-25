library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library source;
use source.all;
library AESLibrary;
use AESLibrary.all;
use AESLibrary.state_definition_package.all;

entity InvShiftRows is
port (	InvShiftRows_i : in type_state;
	InvShiftRows_o : out type_state);
end entity InvShiftRows;


architecture InvShiftRows_arch of InvShiftRows is	--Rotation des données ligne par ligne
	begin
		InvShiftRows_o(0) <= InvShiftRows_i(0)(0) & InvShiftRows_i(0)(1) & InvShiftRows_i(0)(2) & InvShiftRows_i(0)(3);
		InvShiftRows_o(1) <= InvShiftRows_i(1)(3) & InvShiftRows_i(1)(0) & InvShiftRows_i(1)(1) & InvShiftRows_i(1)(2);
		InvShiftRows_o(2) <= InvShiftRows_i(2)(2) & InvShiftRows_i(2)(3) & InvShiftRows_i(2)(0) & InvShiftRows_i(2)(1);
		InvShiftRows_o(3) <= InvShiftRows_i(3)(1) & InvShiftRows_i(3)(2) & InvShiftRows_i(3)(3) & InvShiftRows_i(3)(0);
end architecture InvShiftRows_arch;

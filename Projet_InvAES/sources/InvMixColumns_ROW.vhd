library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
library AESLibrary;
use AESLibrary.state_definition_package.all;

entity InvMixColumns_ROW is
	port (	InvMixColumns_ROW_i : in row_state;
		InvMixColumns_ROW_o : out row_state);
end InvMixColumns_ROW;

--Effectue les opération correspondantes sur une seule ligne de type type_state. Le calcul complet est effectué dans InvMixColumns.vhd.

architecture InvMixColumns_ROW_arch of InvMixColumns_ROW is
	signal s0_s : bit8; 
	signal s0_2s : bit8;
	signal s0_4s : bit8;
	signal s0_8s : bit8;

	signal s1_s : bit8; 
	signal s1_2s : bit8;
	signal s1_4s : bit8;
	signal s1_8s : bit8;

	signal s2_s : bit8; 
	signal s2_2s : bit8;
	signal s2_4s : bit8;
	signal s2_8s : bit8;

	signal s3_s : bit8; 
	signal s3_2s : bit8;
	signal s3_4s : bit8;
	signal s3_8s : bit8;

	signal Coefficients_s : type_state;	--Matrice contenant les produits des multiplications entre les éléments de la matrice et ceux de la colonne (sans les additions du produit matriciel).
	

	begin
	P0 : 
		--Coefficents de la première colonne de la matrice.
		s0_s <= InvMixColumns_ROW_i(0);
			
		s0_2s <= s0_s(6 downto 0) & '0' 	when s0_s(7)='0' 	else (s0_s (6 downto 0 )& '0'  XOR x"1B");	--s0_s * 2
		s0_4s <= s0_2s(6 downto 0) & '0' 	when s0_2s(7)='0' 	else (s0_2s (6 downto 0 )& '0' XOR x"1B");	--s0_s * 4
		s0_8s <= s0_4s(6 downto 0) & '0'	when s0_4s(7)='0' 	else (s0_4s (6 downto 0 )& '0' XOR x"1B");	--s0_s * 8
	
		Coefficients_s(0)(0) <= s0_8s xor s0_4s xor s0_2s;
		Coefficients_s(1)(0) <= s0_8s xor s0_s;
		Coefficients_s(2)(0) <= s0_8s xor s0_4s xor s0_s;
		Coefficients_s(3)(0) <= s0_8s xor s0_2s xor s0_s;

	
		--Coefficents de la deuxième colonne de la matrice.
		s1_s <= InvMixColumns_ROW_i(1);
		
		s1_2s <= s1_s(6 downto 0)  & '0' 	when s1_s(7)='0' 	else (s1_s (6 downto 0 )& '0'  XOR x"1B");
		s1_4s <= s1_2s(6 downto 0) & '0' 	when s1_2s(7)='0' 	else (s1_2s (6 downto 0 )& '0' XOR x"1B");
		s1_8s <= s1_4s(6 downto 0) & '0' 	when s1_4s(7)='0' 	else (s1_4s (6 downto 0 )& '0' XOR x"1B");
	
		Coefficients_s(0)(1) <= s1_8s xor s1_2s xor s1_s;
		Coefficients_s(1)(1) <= s1_8s xor s1_4s xor s1_2s;
		Coefficients_s(2)(1) <= s1_8s xor s1_s;
		Coefficients_s(3)(1) <= s1_8s xor s1_4s xor s1_s;

	
		--Coefficents de la troisième colonne de la matrice.
		s2_s <= InvMixColumns_ROW_i(2);
		
		s2_2s <= s2_s(6 downto 0)  & '0' 	when s2_s(7)='0' 	else (s2_s (6 downto 0 )& '0'  XOR x"1B");
		s2_4s <= s2_2s(6 downto 0) & '0'	when s2_2s(7)='0' 	else (s2_2s (6 downto 0 )& '0' XOR x"1B");
		s2_8s <= s2_4s(6 downto 0) & '0' 	when s2_4s(7)='0' 	else (s2_4s (6 downto 0 )& '0' XOR x"1B");
	
		Coefficients_s(0)(2) <= s2_8s xor s2_4s xor s2_s;
		Coefficients_s(1)(2) <= s2_8s xor s2_2s xor s2_s;
		Coefficients_s(2)(2) <= s2_8s xor s2_4s xor s2_2s;
		Coefficients_s(3)(2) <= s2_8s xor s2_s;

	
		--Coefficents de la quatrième colonne de la matrice.
		s3_s <= InvMixColumns_ROW_i(3);
		
		s3_2s <= s3_s(6 downto 0) & '0' 	when s3_s(7)='0' 	else (s3_s (6 downto 0 )& '0'  XOR x"1B");
		s3_4s <= s3_2s(6 downto 0) & '0' 	when s3_2s(7)='0' 	else (s3_2s (6 downto 0 )& '0' XOR x"1B");
		s3_8s <= s3_4s(6 downto 0) & '0' 	when s3_4s(7)='0' 	else (s3_4s (6 downto 0 )& '0' XOR x"1B");
	
		Coefficients_s(0)(3) <= s3_8s xor s3_s;
		Coefficients_s(1)(3) <= s3_8s xor s3_4s xor s3_s;
		Coefficients_s(2)(3) <= s3_8s xor s3_2s xor s3_s;
		Coefficients_s(3)(3) <= s3_8s xor s3_4s xor s3_2s;
	
		-- On fait la somme des coefficients de la matrice selon les lignes 
		InvMixColumns_ROW_o(0) <= Coefficients_s(0)(0) xor Coefficients_s(0)(1) xor Coefficients_s(0)(2) xor Coefficients_s(0)(3);
		InvMixColumns_ROW_o(1) <= Coefficients_s(1)(0) xor Coefficients_s(1)(1) xor Coefficients_s(1)(2) xor Coefficients_s(1)(3);
		InvMixColumns_ROW_o(2) <= Coefficients_s(2)(0) xor Coefficients_s(2)(1) xor Coefficients_s(2)(2) xor Coefficients_s(2)(3);
		InvMixColumns_ROW_o(3) <= Coefficients_s(3)(0) xor Coefficients_s(3)(1) xor Coefficients_s(3)(2) xor Coefficients_s(3)(3);


end architecture InvMixColumns_ROW_arch;

-- vcom -work source ./sources/InvMixColumns_ROW.vhd

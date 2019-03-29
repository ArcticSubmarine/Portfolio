##
# This program is aimed at simulating a casino roulette.
# The rules are the following : the player bets on a number between 0 and 49 and choose the amount of money he/she wishes to play with.
# The roulette is made of 50 numbers from 0 to 49. Even numbers are black, odd numbers are red. 
# 1°) If the player picked the right number, he/she wins 3 times the money he/she played.
# 2°) Else, if it is the right color, he/she has half of its money back.
# 3°) Else, he/she losts  his/her money.

# Made by Clarisse Tarrou
##


import numpy as np
import random as rd
import sys
import math

#Global variables
money = 1000 #$
player_on = True
right_bet = True
right_number = True
    

while player_on :
    
    response = input("Want to play ? Yes/No       ")
    
    if money == 0 :
        print("You don't have any money left, sorry :(")
        break
        
    
    if response == "Yes" or response == "yes" or response == "Y" or response == "y" :
        bet = input("How much whould you like to bet ?       ")
        
        
        number = input("Which number would you like to bet on ?       ")
        
        bet = int(bet)
        number = int(number)
        
        if bet <= money and number >= 0 and number <= 49 :
        
            result = rd.randrange(50)
            print(result, " !")
            
            #First case
            if number == result :
                win = ceil(3 * bet)
                print('You had the right number ! You just earned ',win, '$.')
                money += win
                
            #Second case
            elif number % 2 == bet % 2 :
                win = bet / 2
                print ('You had the right color ! You just earned', win, '$.')
                money -= win
                
            #Third case
            else :
                print('You lost ... Want to try again ?')
                money -= bet
            
            print("You now have", money, " $.")
            
        else :
            if bet > money :
                print("You don't have enough money to bet ", bet, "$. You actually have ", money, "$.")
            else :
                print("You entered an invalid number. Please choose a number between 0 and 49.")
        
        
    elif response == "No" or response == "no" or response == "N" or response == "n" :
        print("Goodbye !")
        player_on = False
        
    else :
        print("Wrong entry. Please try again")
        

        
    
    
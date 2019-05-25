## This file contains the useful functions for this application.

import random
import os
import pickle

from datas import *

def HaveScores():
    if os.path.exists(SCORE_FILE_NAME):
        scoreFile = open(SCORE_FILE_NAME, "rb")
        depickler = pickle.Unpickler(scoreFile)
        scores = depickler.load()
        scoreFile.close()
    else:
        scores = {}
    return scores

def SaveScore(scores):
    scoreFile = open(SCORE_FILE_NAME, "wb")
    pickler = pickle.Pickler(scoreFile)
    pickler.dump(scores)
    scoreFile.close()

def PlayerName():
    playerName = input("Enter a name : ")
    playerName = playerName.lower()
    if len(playerName) < 4 or not playerName.isalnum():
        print("Invalid nale.")
        return PlayerName()
    else:
        playerName

#Randomly picks a word in the dictionnary.
def WordChoice() :
    length = len(dictionnary)
    n = random.randrange(0,length)
    while n >= length :     #The number picked has to point to something in the list
        n = random.randrange(0,length)
    return dictionnary[n]
    
#Invites the player to chose a letter and verifies its integrity (it has to be low).
def PickLetter() :
    letter = input("Choose a letter : ")
    if 65 <= ord(letter) <= 90 :
        letter = letter.lower()
        return letter
    elif 97 <= ord(letter) <= 122 :
        return letter
    else :
        print("Oops, you chose a wrong entry ({0}). Please try again.".format(letter))
        PickLetter()
        

#Updates list of the letters that have already been found if the letter picked by the player is in the word to find
def UpdateLettersList(wordToFind, pickedLetter, letterList, letterTried) :
    letterTried.append(pickedLetter)
    if pickedLetter in wordToFind :
        if pickedLetter in letterList :
            return False
        else :
            letterList.append(pickedLetter)
            return True
    
    
#Prints the mystery word, with "*" if for letters that have not been found yet
def PrintMysteryWord(wordToFind, letterList) :
    l = []
    for k in range (0, len(wordToFind), 1) :
        if wordToFind[k] in letterList :
            print("{0}".format(wordToFind[k]), end = '')
        else :
            print('*', end = '')
    print("\n\n")
    return 0
    
    
#Print the list of the letters that already been tried
def PrintTriedLetter(letterTried) :
    print("Letters already tried : ", end = '')
    for k in range (0, len(letterTried), 1) :
        print("{0} ".format(letterTried[k]), end = '')
    return 0


#Makes a list of the letters of the word to find
def LettersToFind(wordToFind, lettersToFind) :
    for k in range (0, len(wordToFind), 1) :
        if wordToFind[k] not in lettersToFind :
            lettersToFind.append(wordToFind[k])
    return 0
    
    
#Compares 2 lists and return true if they have the same content (the order of the elements doesn't have to be the same)
def CompareLists(l1, l2) :
    if len(l1) != len(l2) :
        return False
    else :
        for k in range (0, len(l1), 1) :
            if l1[k] not in l2 :
                return False
        return True
        
        
#The game itself        
def MainGame(player, scores) :
    #Initialization
    LETTERSTOFIND = []
    LETTERLIST = []
    LETTERTRIED = []
    LETTER = ''
    WORDTOFIND = ""
    WON = False
    
    WORDTOFIND = WordChoice()
    LettersToFind(WORDTOFIND, LETTERSTOFIND)
    
    
    
    print("Word to find : ")
    PrintMysteryWord(WORDTOFIND, LETTERLIST)
    
    #Main game
    for k in range (0, NUMBER_OF_TRIES, 1) :
        try_nb = 8 - k
        print("\n______________________________________________\n")
        LETTER = PickLetter()   #Pick a letter
        UpdateLettersList(WORDTOFIND, LETTER, LETTERLIST, LETTERTRIED)
        PrintMysteryWord(WORDTOFIND, LETTERLIST)
        PrintTriedLetter(LETTERTRIED)
        print("\n______________________________________________\n\n")
        if CompareLists(LETTERSTOFIND, LETTERLIST) :    #Check if the player won
            WON = True
            break
    
    #Print a message to inform the player about if he/she won or lost
    if WON :
        print("______________________________________________\n______________________________________________\nYou won !\n______________________________________________\n______________________________________________\n")
        scores[player] += 1
    else :
        print("______________________________________________\n______________________________________________\nYou lose !\n______________________________________________\n______________________________________________\n")

def Menu(player, scores) :
    response = input("Want to play ? Yes/No      ")
    response = response.lower()
    if response == "yes" or response == "y" :
        print("Player {0} : {1} point.s.".format(player, scores[player]))
        MainGame(player, scores)
    elif response == "no" or response == "n" :
        print("Goodbye !")
        return 0
    else :
        print("Wrong entry. Please try again")
    Menu(player, scores)
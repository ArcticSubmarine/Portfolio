##
# This file contains an implementation of the hangsman's game.
# The rules are the following :
#   1) the computer choses a word (max. 8 letters) in a pre-defined list ;
#   2) the player tries to find the letters of this word : at each try, he/she chose a letter ;
#   3) if the letter is in the word, the computer displays the word making the already found letters appearing. Those that haven't been found are replaced by stars (*).
#   4) the players have only 8 chances to find the letters or he/she lose the game ;
#   5) at the begining of the game, the player enters his/her name in order to save his/her score.
#   The score is calculated as following : the number of plays resting at a winning party is added to his/her current scoring. For istance, if there are 3 tries left and that the player wins, 3 is added to his/her score.

# The file "datas.py" contains the datas such as the list of words, the number of authorized tries, etc.
# The file functions.py contains the useful functions for this application.
##


# Importation of the needed files and libraries
from datas import *
from functions import *


scores = HaveScores()
user = PlayerName()
if user not in scores.keys():
    scores[user] = 0
Menu(user, scores)


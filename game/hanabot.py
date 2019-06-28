from game import Game
from selfKnowledge import SelfKnowledge

class Hanabot():
    def __init__(self, game, player):
        self._selfknowledge = SelfKnowledge(game, player)


    def decideAction(self, game):

from game import Game, ALL_CARDS
from unknownCard import UnknownCard
from collections import Counter

class SelfKnowledge():
    def __init__(self, game, player):
        self._board = game.board
        self._partnerHand = Counter(game.getHand(1 - player))
        self._hand = Counter(ALL_CARDS)
        self._hand.subtract(self._partnerHand)

    def 
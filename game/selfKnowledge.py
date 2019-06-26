from game import Game, ALL_CARDS
from board import Board
from unknownCard import UnknownCard
from collections import Counter
from consts import NUMBER_IN_HAND
from card import isValidColor, isValidNumber


class SelfKnowledge():
    def __init__(self, game, player):
        self._board = game.board
        self._partnerHand = game.getHand(1 - player)
        self._hand = [UnknownCard() for i in range(NUMBER_IN_HAND)]
        print(self._partnerHand, '\n')
        for card in self._partnerHand:
            for ukCard in self._hand:
                ukCard.exclude(card)

    def updateWithHint(self, feature, indexList):
        assert(isValidColor(feature) or isValidNumber(feature))
        assert(indexList)

        if isValidColor(feature):
            f = lambda card: card.color == feature
        elif isValidNumber(feature):
            f = lambda card: card.number == feature

        for i in range(NUMBER_IN_HAND):
            if i not in indexList:
                nf = lambda card: not (f (card))
                self._hand[i].filter(nf)
            else:
                self._hand[i].filter(f)

    #def updateFromMessage(self, message):
        



    
    def __str__(self):
        s = 'Self knowledge of own hand: \n'
        for unKnownCard in self._hand:
            s += str(unKnownCard) + ',\n'
        return s + '\n'

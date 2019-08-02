from .card import Card
from .consts import NUMBERS, COLORS, AMTS

from collections import Counter

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for _ in range(AMTS[n])]


class UnknownCard():
    """
    A card in a hand.
    """

    def __init__(self):
        self._possibleCards = Counter(ALL_CARDS)
        self._handAge = 0

    @property
    def possible_cards(self):
        return self._possibleCards

    @property
    def handAge(self):
        return self._handAge

    def filter(self, f):
        self._possibleCards = Counter(dict([pair
                                            for pair in self._possibleCards.items() if f(pair[0])]))

    def setDraw(self, drawSet):
        self._possibleCards = drawSet

    def exclude(self, card):
        assert isinstance(card, Card)
        assert self._possibleCards[card]

        self._possibleCards[card] -= 1

    def updateAge(self):
        self._handAge += 1

    def __str__(self):
        return '[Unknown card that could be ' + str(self._possibleCards) + ']'

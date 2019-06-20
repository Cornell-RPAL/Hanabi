from consts import (
    NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS,
    COLORS, NUMBERS, AMTS, NUMBER_IN_HAND,
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)
from card import Card
from collections import Counter
from random import shuffle

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for _ in range(AMTS[n])]

class Board():
    def __init__(self):
        self._hintTokens = NUMBER_OF_HINT_TOKENS
        self._errorTokens = NUMBER_OF_ERROR_TOKENS

        shuffle(ALL_CARDS)
        self._cards = ALL_CARDS
        self._hands = (self._cards[:NUMBER_IN_HAND],
                       self._cards[NUMBER_IN_HAND: NUMBER_IN_HAND * 2])

        self._discardPile = []
        self._drawPile = self._cards[NUMBER_IN_HAND * 2:]

        # self._played_cards = dict(zip(COLORS, [0]*5))
        self._playedPile = []

    @property
    def hintTokens(self):
        return self._hintTokens

    @hintTokens.setter
    def hintTokens(self, val):
        assert val >= 0
        self._hintTokens = val

    @property
    def errorTokens(self):
        return self._errorTokens

    @errorTokens.setter
    def errorTokens(self, val):
        assert val >= 0
        self._errorTokens = val

    @property
    def discardPile(self):
        return self._discardPile

    @discardPile.setter
    def discardPile(self, val):
        self._discardPile = val

    @property
    def drawPile(self):
        return self._drawPile

    @drawPile.setter
    def drawPile(self, val):
        self._drawPile = val

    @property
    def playedPile(self):
        return self._playedPile

    @playedPile.setter
    def playedPile(self, val):
        self._playedPile = val

    


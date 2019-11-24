from .consts import (
    NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS, COLORS
)
from .card import Card
from collections import Counter
from random import shuffle


class Board():
    """
    The physical board state that Hanabot can observe (mainly through CV).
    """
    def __init__(self, partnerHand):
        self._hintTokens = NUMBER_OF_HINT_TOKENS
        self._errorTokens = NUMBER_OF_ERROR_TOKENS

        self._partnerHand = partnerHand # initialize partnerHand from CV

        self._discardPile = []
        self._playedPile = []

        self._cardInGripper = None

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

    @property
    def cardInGripper(self):
        return self._cardInGripper
        
    @cardInGripper.setter
    def cardInGripper(self, val):
        self._cardInGripper = val

    @property
    def partnerHand(self):
        return self._partnerHand

    @partnerHand.setter
    def partnerHand(self, val):
        self._partnerHand = val

    def topPlayedCards(self):
        """
        A dictionary representing the top card of each color. 

        The value is [0] if there is no cards of the color.
        """
        d = dict(zip(COLORS, [0]*5))
        for c in self.playedPile:
            if d[c.color] < c.number:
                d[c.color] = c.number
        return d


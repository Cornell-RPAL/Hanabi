from .board import Board
from .unknownCard import UnknownCard, ALL_CARDS
from collections import Counter
from .consts import (
    NUMBER_IN_HAND, HANABOT, NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS
)
from .card import Card, isValidColor, isValidNumber
from .action import PlaySuccess, PlayFail, Discard, Hint


class SelfKnowledge():
    def __init__(self, board, player, partnerKnowledge=None):
        self._board = board
        self._player = player
        self._hintTokens = NUMBER_OF_HINT_TOKENS
        self._errorTokens = NUMBER_OF_ERROR_TOKENS
        self._discarded = self._board.discardPile
        self._played = self._board.playedPile
        self._hand = [UnknownCard() for _ in range(NUMBER_IN_HAND)]
        self._drawCards = Counter(ALL_CARDS)
        if player == HANABOT:
            self._partnerHand = board.partnerHand
            self._partnerKnowledge = SelfKnowledge(board, 1 - HANABOT, \
                partnerKnowledge=self)
            for card in self._partnerHand:
                self._drawCards[card] -= 1
                for ukCard in self._hand:
                    ukCard.setDraw(self._drawCards)
        else: self._partnerKnowledge = partnerKnowledge

    @property
    def board(self):
        return self._board
    
    @property
    def partnerHand(self):
        return self._partnerHand

    @property
    def hand(self):
        return self._hand

    def updateHandAge(self):
        for ukcard in self._hand:
            ukcard.updateAge()

    def excludeCard(self, card):
        if self._drawCards[card]:
            self._drawCards[card] -= 1
        for ukCard in self._hand:
            if ukCard._possibleCards[card]:
                ukCard.exclude(card)

    def updateHelper(self, action):
        card = self._partnerHand[action.indices[0]]
        self.excludeCard(card)

    def updateSelfAction(self, action):
        if self._player != HANABOT:
            self._partnerKnowledge.updateHelper(action)
        card = action.card
        if isinstance(action, PlayFail):
            self._errorTokens -= 1
        if self._drawCards[card] and self._player == HANABOT:
            self._drawCards[card] -= 1
        self._hand[action.indices[0]].setDraw(self._drawCards)

    def updatePartnerAction(self, action):
        self._partnerKnowledge.updateSelfAction(action)

    def updateWithHint(self, feature, indexList):
        assert(isValidColor(feature) or isValidNumber(feature))
        assert(indexList)
        assert(self._hintTokens > 0)

        if isValidColor(feature):
            f = lambda card: card.color == feature
        elif isValidNumber(feature):
            f = lambda card: card.number == feature
        
        self._hintTokens -= 1
        for i in range(NUMBER_IN_HAND):
            if i not in indexList:
                nf = lambda card: not (f (card))
                self._hand[i].filter(nf)
            else:
                self._hand[i].filter(f)

    def updatePartnerWithHint(self, feature, indexList):
        self._partnerKnowledge.updateWithHint(feature, indexList)

    def getPartnerHandKnowledge(self):
        return self._partnerKnowledge.hand
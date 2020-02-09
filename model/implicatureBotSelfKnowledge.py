from .board import Board
from .unknownCard import UnknownCard, ALL_CARDS
from collections import Counter
from .consts import (
    NUMBER_IN_HAND, HANABOT, NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS
)
from .card import Card, isValidColor, isValidNumber
from .action import PlaySuccess, PlayFail, Discard, Hint
from .selfKnowledge import SelfKnowledge


class implicatureBotSelfKnowledge():
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
        #print(action.indices[0])
        #print(self._partnerHand)
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

    def updateBeliefHelper(self, myCardIndex, feature, indexList):
        #print("in outer of update belief helper")
        #print("feature is", feature)
        top = self._board.topPlayedCards()
        #print("top", top)
        if(len(indexList) == 1):
            if isValidColor(feature):
                nextOfColor = top[feature]+1
                self._hand[i].filterBeliefsForImplicature(feature, nextCardInColor = nextOfColor)
            elif isValidNumber(feature):
                #print("in update belief helper")
                for key in top:
                    num = top.get(key) + 1
                    if(num == feature):
                        self._hand[myCardIndex].filterBeliefsForImplicature(feature, color = key)


    def updateWithHint(self, feature, indexList):
        assert(isValidColor(feature) or isValidNumber(feature))
        assert(indexList)
        assert(self._hintTokens > 0)

        if isValidColor(feature):
            f = lambda card: card.color == feature
        elif isValidNumber(feature):
            f = lambda card: card.number == feature

        self._hintTokens -= 1
        #print("index 1 had before", self._hand[1])
        for i in range(NUMBER_IN_HAND):
            if i not in indexList:
                nf = lambda id: not feature
                self._hand[i].possible['colors'] = list(filter(nf, self._hand[i].possible['colors']))
                self._hand[i].possible['numbers'] = list(filter(nf, self._hand[i].possible['numbers']))
            else:
                #print("feature: ", feature)
                #print("original hand", self._hand[i])
                self._hand[i]._possibleCards = Counter(filter(f, self._hand[i]._possibleCards.elements()))
                self._hand[i].filterPossibiliitiesAndBeliefs(feature)
                #print("after filter Possibilities and Beliefs, possible numbers ", self._hand[i].possible['numbers'])
                #print("after filter Possibilities and Beliefs, believed numbers ", self._hand[i].beliefs['numbers'])
                self.updateBeliefHelper(i, feature, indexList)
        #print("index 1 hand afterwards believed", self._hand[1].beliefs)


    def updatePartnerWithHint(self, feature, indexList):
        self._partnerKnowledge.updateWithHint(feature, indexList)

    def getPartnerHandKnowledge(self):
        return self._partnerKnowledge.hand

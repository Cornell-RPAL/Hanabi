from game import Game, ALL_CARDS
from board import Board
from unknownCard import UnknownCard
from collections import Counter
from consts import NUMBER_IN_HAND, NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS
from card import isValidColor, isValidNumber
from action import Action, PlayCard, Discard, Hint


class SelfKnowledge():
    def __init__(self, game, player, layer, opp_knowledge=None):
        self._board = game.board
        self._player = player
        self._game = game
        self._partnerHand = game.getHand(1 - player)
        self._hintTokens = self._board._hintTokens
        self._errorTokens = self._board._errorTokens
        self._discarded = self._board._discardPile
        self._played = self._board._playedPile
        self._hand = [UnknownCard() for _ in range(NUMBER_IN_HAND)]
        # self._curHints = [(-1, '') for _ in range(NUMBER_IN_HAND)]
        self._drawSet = Counter(ALL_CARDS)
        if layer == 0:
            self._oppKnowledge = SelfKnowledge(game, (1-player), layer = 1, opp_knowledge=self)
        # #print(self._partnerHand, '\n')
        for card in self._partnerHand:
            self._drawSet[card] -= 1
            for ukCard in self._hand:
                ukCard.setDraw(self._drawSet)

    @property
    def board(self):
        return self._board
    
    @property
    def partnerHand(self):
        return self._game.getHand(1 - self._player)

    @property
    def hand(self):
        return self._hand

    # def hintReceived(self):


    # # def newKnowledge(self):
    
    def updateHandAge(self):
        for ukcard in self._hand:
            ukcard.updateAge()

    def excludeCard(self, card):
        if self._drawSet[card]:
            self._drawSet[card] -= 1
        for ukCard in self._hand:
            if ukCard._possibleCards[card]:
                ukCard.exclude(card)

    def updateSelfAction(self, action):
        card_ix = action.card_ix
        if isinstance(action, PlayCard):
            last_card = self._played[-1]
        else:
            last_card = self._discarded[-1]
        if self._drawSet[last_card]:
            self._drawSet[last_card] -= 1
        self._hand[card_ix].setDraw(self._drawSet)

    def updateOppAction(self):
        for card in self._partnerHand:
            self.excludeCard(card)

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

    def updateOppWithHint(self, feature, indexList):
        self._oppKnowledge.updateWithHint(feature, indexList)

    @property
    def getOppHandKnowledge(self):
        return self._oppKnowledge._hand

    def __str__(self):
        s = 'Self knowledge of own hand: \n'
        for unKnownCard in self._hand:
            s += str(unKnownCard) + ',\n'
        return s + '\n'

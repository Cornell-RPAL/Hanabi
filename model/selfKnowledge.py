from .game import Game, ALL_CARDS
from .board import Board
from .unknownCard import UnknownCard
from collections import Counter
from .consts import NUMBER_IN_HAND, HANABOT, NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS
from .card import isValidColor, isValidNumber
from .action import Action, PlayCard, Discard, Hint

class SelfKnowledge():
    def __init__(self, game, player, opp_knowledge=None):
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
        self._drawCards = Counter(ALL_CARDS)
        if player == HANABOT:
            self._oppKnowledge = SelfKnowledge(game, (1-player), opp_knowledge=self)
        else: self._oppKnowledge = opp_knowledge
        for card in self._partnerHand:
            self._drawCards[card] -= 1
            for ukCard in self._hand:
                ukCard.setDraw(self._drawCards)

    @property
    def board(self):
        return self._board
    
    @property
    def partnerHand(self):
        return self._game.getHand(1 - self._player)

    @property
    def hand(self):
        return self._hand

    # # def newKnowledge(self):
    
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
        card = self._partnerHand[action.card_ix]
        self.excludeCard(card)

    def updateSelfAction(self, action):
        self._oppKnowledge.updateHelper(action)
        if isinstance(action, PlayCard):
            last_card = self._played[-1]
        else:
            last_card = self._discarded[-1]
        if self._drawCards[last_card]:
            self._drawCards[last_card] -= 1
        self._hand[action.card_ix].setDraw(self._drawCards)

    def updateOppAction(self, action):
        self._oppKnowledge.updateSelfAction(action)

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

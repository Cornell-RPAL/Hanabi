import random
from .game import Game
from .selfKnowledge import SelfKnowledge
from .action import Action, Play, Discard, Hint
from .board import Board, ALL_CARDS
from .consts import NUMBER_IN_HAND, HANABOT
from .unknownCard import UnknownCard

class Hanabot():
    def __init__(self, board):
        self._board = board
        self._selfknowledge = SelfKnowledge(game, HANABOT)
        self._player = HANABOT

    def isPlayable(self, card):
        top = self._board.topPlayedCards()
        if isinstance(card, UnknownCard):
            for possibleCard in card._possibleCards:
                num = top[possibleCard.color]
                if num != possibleCard.number - 1:
                    return False
            return True
        else:
            num = top.get(card.color)
            return (num == card.number - 1)

    def playables(self, hand):
        return [i for i, card in enumerate(hand) if self.isPlayable(card)]

    def isDiscardable(self, card):
        if isinstance(card, UnknownCard):
            for possible_card in card._possibleCards:
                if possible_card not in self._selfknowledge._played:
                    return False
            return True
        return card in self._selfknowledge._played

    def discardables(self, hand):
        return [i for i, card in enumerate(hand) if self.isDiscardable(card)]

    def checkDup(self, hand):
        dups = []
        s = set()
        for i in range(NUMBER_IN_HAND):
            card = hand[i]
            try:
                if card in s:
                    dups.append(card)
            except Exception : continue
            s.add(card)
        return dups

    def partnerFeatures(self):
        friendHand = self._selfknowledge.partnerHand
        allFeatures = [card.number for card in friendHand] + \
                [card.color for card in friendHand]

        return list(set(allFeatures))

    def hintRandom(self, hintList):
        return random.choice(hintList)

    def getPlayableHint(self):
        oppHand = self._selfknowledge.partnerHand
        playables = self.playables(oppHand)
        if playables:
            card = oppHand[random.choice(playables)]
            return random.choice([card.color, card.number])
        else: 
            return None

    def hintCmp(self, hintList):
        playableHint = self.getPlayableHint()
        return playableHint or self.hintRandom(hintList)

    def fullHint(self):
        hintList = self.partnerFeatures()
        return Hint(self._player, feature=self.hintCmp(hintList))

    def discardRandom(self):
        hand = self._selfknowledge.hand
        return Discard(self._player, random.randrange(len(hand)))

    def getBasicAction(self):
        hand = self._selfknowledge.hand

        playables = self.playables(hand)
        discardables = self.discardables(hand)

        if playables:
            return Play(self._player, random.choice(playables))
        elif discardables:
            return Discard(self._player, random.choice(discardables))
        else:
            return None

    def decideAction(self):
        partnerHandKnowledge = self._selfknowledge.getPartnerHandKnowledge()
        hintList = self.partnerFeatures()
        basicAction = self.getBasicAction()
        if basicAction:
            return basicAction
        elif (self.playables(partnerHandKnowledge) \
            or self.discardables(partnerHandKnowledge)):
            return self.fullHint()
        elif self._board.hintTokens > 0:
            playableHint = self.getPlayableHint()
            if playableHint:
                return Hint (self._player, feature = playableHint)
            return Hint(self._player, feature = self.hintRandom(hintList))
        else:
            return self.discardRandom()
        
    def inform(self, action):
        self._selfknowledge.updateHandAge()
        if action.player_num == HANABOT:
            self._selfknowledge.updateSelfAction(action)
        else: 
            self._selfknowledge.updateOppAction(action)

    def informHint(self, action, il):
        self._selfknowledge.updateHandAge()
        if action.player_num == HANABOT:
            self._selfknowledge.updateOppWithHint(action.feature, il)
        else: 
            self._selfknowledge.updateWithHint(action.feature, il)


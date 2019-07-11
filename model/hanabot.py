import random
from .game import Game
from .selfKnowledge import SelfKnowledge
from .action import Action, PlayCard, Discard, Hint
from .board import Board, ALL_CARDS
from .consts import NUMBER_IN_HAND, HANABOT
from .unknownCard import UnknownCard

class Hanabot():
    def __init__(self, game):
        self._game = game
        self._selfknowledge = SelfKnowledge(game, HANABOT)
        self._player = HANABOT

    def playable(self, card):
        top = self._game.topPlayedCards()
        if isinstance(card, UnknownCard):
            for possible_card in card._possibleCards:
                num = top.get(possible_card.color)
                if num != possible_card.number-1:
                    return False
            return True
        num = top.get(card.color)
        return num == card.number-1

    def playables(self, hand):
        playables = []
        for j in range(len(hand)):
            card = hand[j]
            if self.playable(card):
                playables.append(j)
        return playables

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

    def discardableCard(self, card):
        if isinstance(card, UnknownCard):
            for possible_card in card._possibleCards:
                if possible_card not in self._selfknowledge._played:
                    return False
            return True
        return card in self._selfknowledge._played

    def discardable(self, hand):
        discardables = self.checkDup(hand)
        for j in range(len(hand)):
            card = hand[j]
            if self.discardableCard(card):
                discardables.append(j)
        return discardables

    def generateHint(self):
        oppHand = self._selfknowledge.partnerHand

        feature_list = []
        for card in oppHand:
            if card.number not in feature_list:
                feature_list.append (card.number)
            if card.color not in feature_list:
                feature_list.append (card.color)
        return feature_list

    def hintRandom(self, hint_list):
        return random.choice(hint_list)

    def hintPlayable(self):
        oppHand = self._selfknowledge._partnerHand
        playables = self.playables(oppHand)
        if playables:
            card = oppHand[random.choice(playables)]
            return random.choice([card.color, card.number])
        else: return None

    def hintCmp(self, hint_list):
        playableHint = self.hintPlayable()
        if playableHint != None:
            return playableHint
        return self.hintRandom(hint_list)

    def fullHint(self):
        hint_list = self.generateHint()
        return Hint(self._player, feature=self.hintCmp(hint_list))

    def discardRandom(self):
        hand = self._selfknowledge._hand
        return Discard(self._player, random.randrange(len(hand)))

    def basicAction(self, mode, hint_list=None):
        hand = self._selfknowledge._hand
        playables = self.playables(hand)
        if playables:
            return PlayCard(self._player, random.choice(playables))
        discardables = self.discardable(hand)
        if discardables:
            return Discard(self._player, random.choice(discardables))
        if mode == 1:
            return self.fullHint()
        if mode == 2:
            return Hint(self._player, feature = self.hintRandom(hint_list))
        if mode == 3:
            return self.discardRandom()

    def decideAction(self):
        #know = self._selfknowledge._hand
        opp_knowledge = self._selfknowledge.getOppHandKnowledge
        hint_list = self.generateHint()
        if self.playables(opp_knowledge) or self.discardable(opp_knowledge):
            return self.basicAction(1, hint_list)
        if self._selfknowledge._hintTokens > 0:
            playableHint = self.hintPlayable()
            if playableHint != None:
                return Hint (self._player, feature=playableHint)
            return self.basicAction(2, hint_list)
        return self.basicAction(3)
        
    def inform(self, action):
        self._selfknowledge.updateHandAge()
        if action.player_num == HANABOT:
            self._selfknowledge.updateSelfAction(action)
        else: self._selfknowledge.updateOppAction(action)

    def informHint(self, action, il):
        self._selfknowledge.updateHandAge()
        if action.player_num == HANABOT:
            self._selfknowledge.updateOppWithHint(action.feature, il)
        else: self._selfknowledge.updateWithHint(action.feature, il)


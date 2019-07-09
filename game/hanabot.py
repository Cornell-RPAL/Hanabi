import random
from game import Game
from selfKnowledge import SelfKnowledge
from action import Action, PlayCard, Discard, Hint
from board import Board, ALL_CARDS
from consts import NUMBER_IN_HAND
from unknownCard import UnknownCard

class Hanabot():
    def __init__(self, game, player):
        self._game = game
        self._selfknowledge = SelfKnowledge(game, player, layer = 0)
        self._player = player
        self._possibleHints = []

    def inform(self, action, il=None):
        if action.player_num == self._player:
            if isinstance(action, Hint):
                self._selfknowledge.updateOppWithHint(action.feature, il)
            else: self._selfknowledge.updateSelfAction(action)
        elif isinstance(action, Hint):
                self._selfknowledge.updateWithHint(action.feature, il)
        else: self._selfknowledge.updateOppAction()

    def playable(self, card):
        if isinstance(card, UnknownCard):
            for possible_card in card._possibleCards:
                num = self._game.topPlayedCards().get(possible_card.color)
                if num != possible_card.number-1:
                    return False
            return True
        num = self._game.topPlayedCards().get(card.color)
        return num == card.number-1

    def playables(self, hand):
        playables = []
        for j in range(len(hand)):
            card = hand[j]
            if self.playable(card):
                playables.append(j)
        return playables

    def checkDup(self, hand):
        for i in range(NUMBER_IN_HAND):
            for j in range(i+1, NUMBER_IN_HAND):
                try:
                    if hand[i].color==hand[j].color and hand[i].number == hand[j].number:
                        return i
                except Exception : return -1
        return -1

    def discardableCard(self, card):
        if isinstance(card, UnknownCard):
            for possible_card in card._possibleCards:
                if possible_card not in self._selfknowledge._played:
                    return False
            return True
        return card in self._selfknowledge._played

    def discardable(self, hand):
        discardables = []
        i = self.checkDup(hand)
        if i != -1:
            discardables.append(hand[i])
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
        return Hint(self._game, self._player, feature=self.hintCmp(hint_list))

    def discardRandom(self):
        hand = self._selfknowledge._hand
        return Discard(self._game, self._game, random.randrange(len(hand)))

    def basicAction(self, mode, hint_list=None):
        hand = self._selfknowledge._hand
        playables = self.playables(hand)
        if playables:
            return PlayCard(self._game, self._player, random.choice(playables))
        discardables = self.discardable(hand)
        if discardables:
            return Discard(self._game, self._player, random.choice(discardables))
        if mode == 1:
            return self.fullHint()
        if mode == 2:
            return self.hintRandom(hint_list)
        if mode == 3:
            return self.discardRandom()

    def decideAction(self):
        opp_knowledge = self._selfknowledge.getOppHandKnowledge
        hint_list = self.generateHint()
        if self.playables(opp_knowledge) or self.discardable(opp_knowledge):
            return self.basicAction(1, hint_list)
        if self._selfknowledge._hintTokens > 0:
            playableHint = self.hintPlayable()
            if playableHint != None:
                return Hint(self._game, self._player, feature=playableHint)
            return self.basicAction(2, hint_list)
        return self.basicAction(3)
        


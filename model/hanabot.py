import random
import asyncio
from .selfKnowledge import SelfKnowledge
from .intent import PlayIntent, DiscardIntent, HintIntent
from .action import PlaySuccess, PlayFail, Discard, Hint
from .board import Board
from .consts import NUMBER_IN_HAND, HANABOT
from .unknownCard import UnknownCard
from .message import Message
from log import log

class Hanabot():
    def __init__(self, board):
        self._board = board # stores pointer so always updated with reality
        self._selfknowledge = SelfKnowledge(board, HANABOT)
        self._player = HANABOT
        self._currentIntent = None

    async def react(self, iBuffer, oBuffer):
        while True:
            await asyncio.sleep(0.05)
            # react to player action
            observedAction = iBuffer.action
            log(observedAction and observedAction.indices)
            if observedAction:
                self.inform(observedAction)

                intent = self.decideAction()
                self._currentIntent = intent
                oBuffer.intent = intent
                oBuffer.baxterCommand = intent.baxterCommand

                if isinstance(self._currentIntent, HintIntent):
                    cards = [card for card in self._selfknowledge.partnerHand \
                        if card.hasFeature(self._currentIntent.feature)]
                    self.inform(self._currentIntent.complete(cards))
                    self._currentIntent = None

                m = Message()
                text = m.respond(intent)
                oBuffer.text = text

            # react to self action (intent)
            card = iBuffer.getGripper()
            if card:
                log('AI recognized card in gripper')
                log(self._currentIntent)
                action = None
                if isinstance(self._currentIntent, PlayIntent):
                    if self.isPlayable(card):
                        log('card is playable')
                        action = self._currentIntent.complete([card], success=True)
                        oBuffer.baxterCommand = ("play", [self.playableIndex(card)])
                    else:
                        action = self._currentIntent.complete([card], success=False)
                        oBuffer.baxterCommand = ("discard", [])
                elif isinstance(self._currentIntent, DiscardIntent):
                    action = self._currentIntent.complete([card])
                    oBuffer.baxterCommand = ("discard", [])

                if action:
                    self.inform(action)
                self._currentIntent = None

    def playableIndex(self, card):
        top = self._board.topPlayedCards()


        for i, topCardColor in enumerate(top):
            if card.color == topCardColor:
                return i
        raise Exception("Card not playable!")


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
        partnerHand = self._selfknowledge.partnerHand
        playables = self.playables(partnerHand)
        if playables:
            card = partnerHand[random.choice(playables)]
            return random.choice([card.color, card.number])
        else:
            return None

    def hintCmp(self, hintList):
        playableHint = self.getPlayableHint()
        return playableHint or self.hintRandom(hintList)

    def indicesOfFeature(self, feature):
        return [i for i, card in enumerate(self._selfknowledge.partnerHand) \
            if card.hasFeature(feature)]

    def fullHint(self):
        hintList = self.partnerFeatures()
        feature = self.hintCmp(hintList)
        indices = self.indicesOfFeature(feature)
        return HintIntent(feature, indices)

    def discardRandom(self):
        hand = self._selfknowledge.hand
        return DiscardIntent(indices=[random.randrange(len(hand))])

    def getBasicAction(self):
        hand = self._selfknowledge.hand

        playables = self.playables(hand)
        discardables = self.discardables(hand)

        if playables:
            return PlayIntent(indices=[random.choice(playables)])
        elif discardables:
            return DiscardIntent(indices=[random.choice(discardables)])
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
            feature = playableHint or self.hintRandom(hintList)
            indices = self.indicesOfFeature(feature)
            return HintIntent(feature = feature, indices = indices)

        else:
            return self.discardRandom()


    def inform(self, action):
        self._selfknowledge.updateHandAge()
        if isinstance(action, Hint):
            if action.player == HANABOT:
                self._selfknowledge.updatePartnerWithHint(action.feature, action.indices)
            else:
                self._selfknowledge.updateWithHint(action.feature, action.indices)
        elif action.player == HANABOT:
            self._selfknowledge.updateSelfAction(action)
        else:
            self._selfknowledge.updatePartnerAction(action)

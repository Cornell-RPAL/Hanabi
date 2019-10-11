import random
import asyncio
from .selfKnowledge import SelfKnowledge
from .intent import PlayIntent, DiscardIntent, HintIntent
from .action import PlaySuccess, PlayFail, Discard, Hint
from .board import Board
from .consts import NUMBER_IN_HAND, HANABOT
from .unknownCard import UnknownCard
from .message import Message

class simpleImplicatureBot():
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
            print(observedAction and observedAction.indices)
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
                print('AI recognized card in gripper')
                print(self._currentIntent)
                action = None
                if isinstance(self._currentIntent, PlayIntent):
                    if self.isPlayable(card):
                        print('card is playable')
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

    def indicesOfFeature(self, feature):
        return [i for i, card in enumerate(self._selfknowledge.partnerHand) \
            if card.hasFeature(feature)]

    def indicesOfMyNumber(self. number):
        return [i for i, card in enumerate(self._selfknowledge.Hand) \
            if card.number == number]

    def discardables(self, hand):
        return [i for i, card in enumerate(hand) if self.isDiscardable(card)]

    def myImmediatelyPlayableBeleif(self): #change for beliefs vs possibility
        hand = self._selfknowledge.hand
        for i in range(len(self._selfknowledge.hand)):
            if(len(hand[i].beliefs["number"]) == 1 and len(hand[i].beliefs["color"]) == 1): #if certain about a card
                if(topPlayedCards(board)[card.color] != 0): #if the color has been played before
                    if(isPlayable(card)):#if card playable, return index of card
                        return i
                elif hand[i].beliefs["number"][0] == 1:#else if you think it is a 1
                    return i #return index of card
        return None

    def teamMateImmediatelyPlayable(self):
        return playables(board.partnerHand)

    def teamMateImmediatelyPlayableUnknown(self):
        ret = []
        hand = self._selfKnowledge.hand
        for i in teamMateImmediatelyPlayable():
            if hand[i].possibilities["number"] > 1 and hand[i].possibilites["color"] > 1
                ret.append(i)
        return ret

    def activeFireworks(self):
        counter = 0
        for x, y in board.topPlayedCards():
            if y > 0:
                counter += 1
        return counter

    def checkNotPlayedp2for1(self, num):
        ret = None
        lst = indicesOfMyNumber(num)
        for x in lst:
            card = self._selfKnowledge.hand[x].possibility["color"] #assumes 0 is removed!
            if(not (len(cards) == 1 and board.topPlayedCards[cards[0].color] == 0)):
                ret = x
        return x

    def minNumber(self):
        track = 6
        for c, x in topPlayedCards:
            if x < track:
                track = x
        return track

    def checkDeadByNumber(number, pile):
        f = lambda card: card.number == number
        return len(filter(filterFunction(number), pile)) == AMTS[number]

    def deadFireworkOfColor(color):
        f = lambda card: card.color == color
        discardOfColor = filter(f, board.discardPile)
        for i in range(5):
            if(checkDeadByNumber(i, discardOfColor)):
                return i
        return None

    def altanySureDiscard():
        checkedDup = []
        for i in range(len(selfKnowledge.hand)):
            x = selfKnowledge.hand[i]
            if (len(x.beliefs["colors"]) == 1 and len(x.beliefs["number"]) == 1 and x.beliefs["number"][0] <= board.topPlayedCards[x.color]):
                return i
            elif(len(x.beliefs["number"]) == 1 and len(x.beliefs["number"][0]) < minNumber()):
                return i
            elif(len(x.beliefs["colors"]) == 1 and board.topPlayedCards[x.beliefs["colors"][0]] == 5):
                return i
            elif(len(x.beliefs["colors"]) == 1 and min(x.beliefs["number"]) > deadFireworkOfColor(x.beliefs["color"][0])):
                return i
            #TODO override __eq__ methods
        elif(len(x.beliefs["colors"] == 1) and len(x.beliefs["number"] == 1)):
                if (x.beliefs["colors"][0], x.beliefs.number[0]) in checkedDup:
                    return i
                else:
                    checkedDup.append((x.beliefs["colors"][0], x.beliefs.number[0]))
        return None

    def anyNoPartial(self):
        for i in len(range(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]
            if(len(x.possibility["number"]) > 1 and len(x.possibility["color"]) > 1):
                return i
        return None

    def anyNoFive(self):
        for i in range(len(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]
            if(!(len(x.possibility["number"]) == 1 and x.possibility["number"][0] == 5):
                return i
        return None

    def lessRiskyDiscard(self):
        if(anyNoPartial(self)):
            return anyNoPartial(self)
        else if(anyNoFive(self)):
            return anyNoFive(self)
        else:
            return self._selfKnowledge.hand[randint(0, 4)]


    def anyNoInfo(self): #possibilities is only updated with hints
        for i in range(len(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]
            if (len(x.possibility.color.length == 5 and x.possibilities.number.length == 5)):
                return i
        return None


    def decideAction(self):
        #need beliefs of cards
        #beliefs are updated in updateBeliefs. 1 hint of color sets belief of the number
            #to the next number. 1 hint of number, if it matches a color stack and unsure
            #about the color of that card, then set color to where it would be viable to play
        #game.played is array indexed by color of queues? of played cards
        if(immediatelyPlayable()): #any of my cards are playable
            return PlayIntent(indices=[immediatelyPlayable(self)])
        elif(teamMateImmediatelyPlayableUnknown() > 1 && len(playedPile) == 0): #teammate's cards are immediately playable, know nothing about, and score = 0
            card = teamMateImmediatelyPlayableUnkown[0] #TODO is htis correct logic?
            return HintIntent(indices = indicesOfFeature(card.color) feature = card.color)
        elif(activeFireworks < 4 && checkNotPlayedp2for1(1)): #less than or equal to 4 stacks played and the 1 of color is not played yet
            return HintIntent(indices = checkNotPlayedp2for1(1), feature = 1)
        elif(altanySureDiscard(self)): #no hints left and sure of a discard
            return DiscardIntent(indices=[altanySureDiscard(self)])
        elif(board.hintTokens < 1 and anyNoInfo(self)): #no hints and cards with no info, discard it
            return DiscardIntent(indices = [anyNoInfo(self)])
        elif(board.hintTokens <= 1): #1 or less hints, do less risky discard -
            return DiscardIntent(indices = [lessRiskyDiscard(self)])

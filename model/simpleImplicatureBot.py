import random
import asyncio
from .implicatureBotSelfKnowledge import implicatureBotSelfKnowledge
from .selfKnowledge import SelfKnowledge
from .intent import PlayIntent, DiscardIntent, HintIntent
from .action import PlaySuccess, PlayFail, Discard, Hint
from .board import Board
from .consts import NUMBER_IN_HAND, HANABOT
from .unknownCard import UnknownCard
from .message import Message

class simpleImplicatureBot():
    def __init__(self, board, selfknowledge):
        self._board = board # stores pointer so always updated with reality
        self._selfKnowledge = selfknowledge #implicatureBotSelfKnowledge(board, HANABOT)
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
                    cards = [card for card in self._selfKnowledge.partnerHand \
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
                if possible_card not in self._selfKnowledge._played:
                    return False
            return True
        return card in self._selfKnowledge._played

    def indicesOfFeature(self, feature):
        return [i for i, card in enumerate(self._selfKnowledge.partnerHand) \
            if card.hasFeature(feature)]

    def indicesOfMyNumber(self, number):
        return [i for i, card in enumerate(self._selfKnowledge._hand) \
            if len(card.possible["numbers"]) == 1 and  \
            card.possible["numbers"][0] == number]

    def discardables(self, hand):
        return [i for i, card in enumerate(hand) if self.isDiscardable(card)]

    def myImmediatelyPlayableBelief(self): #change for beliefs vs possibility
        hand = self._selfKnowledge.hand
        #print("HAND!!!:", self._selfKnowledge.hand[0], self._selfKnowledge.hand[1], self._selfKnowledge.hand[2], self._selfKnowledge.hand[3], self._selfKnowledge.hand[4])
        for i in range(len(self._selfKnowledge.hand)):
            #print(i)
            #print("in loop")
            #print(hand[i].beliefs["colors"])
            #print(hand[i].beliefs["numbers"])
            card = hand[i]
            if(len(hand[i].beliefs["numbers"]) == 1 and len(hand[i].beliefs["colors"]) == 1): #if certain about a card
                #print("in if")
                if(self._board.topPlayedCards()[card.beliefs["colors"][0]] != 0): #if the color has been played before
                    if(self.isPlayableBelief(card)):#if card playable, return index of card
                        return i
                elif hand[i].beliefs["numbers"][0] == 1:#else if you think it is a 1
                    return i #return index of card
        return None

    def isPlayableBelief(self, card):
        top = self._board.topPlayedCards()
        if (top[card.beliefs['colors'][0]] + 1) == card.beliefs['numbers'][0]:
            return True
        return False

    def teamMateImmediatelyPlayable(self):
        return self.playables(self._board.partnerHand)

    def teamMateImmediatelyPlayableUnknown(self):
        ret = []
        hand = self._selfKnowledge._partnerKnowledge.hand
        for i in range(0, len(self.teamMateImmediatelyPlayable())):
            #print("for card index i in teamate's hand ", i)
            #print("possible numbers are ", hand[i].possible['numbers'])
            #print('possible colors are ', hand[i].possible['colors'])
            if len(hand[i].possible["numbers"]) > 1 and len(hand[i].possible["colors"]) > 1:
                #print("appended to list: index ", i)
                ret.append(i)
        return ret

    def activeFireworks(self):
        ret = []
        for color in self._board.topPlayedCards():
            if self._board.topPlayedCards()[color] < 1:
                ret.append(color)
        print("number of active fireworks is", len(ret))
        return len(ret)

    def checkNotPlayedMyCardsFor1(self, num):
        lst = self.indicesOfMyNumber(num)
        for x in lst:
            card_colors = self._selfKnowledge.hand[x].possible["colors"] #assumes 0 is removed!
            if(not (len(card_colors) == 1 and self._board.topPlayedCards[card_colors[0]] == 0)):
                return x
        return None

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
        discardOfColor = list(filter(f, self._board.discardPile))
        for i in range(5):
            if(checkDeadByNumber(i, discardOfColor)):
                return i
        return None

    def altanySureDiscard():
        checkedDup = []
        for i in range(len(selfKnowledge.hand)):
            x = selfKnowledge.hand[i]
            if (len(x.beliefs["colors"]) == 1 and len(x.beliefs["number"]) == 1 and x.beliefs["number"][0] <= self._board.topPlayedCards[x.color]):
                return i
            elif(len(x.beliefs["number"]) == 1 and len(x.beliefs["number"][0]) < minNumber()):
                return i
            elif(len(x.beliefs["colors"]) == 1 and self._board.topPlayedCards[x.beliefs["colors"][0]] == 5):
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
            if(not (len(x.possibility["number"]) == 1 and x.possibility["number"][0] == 5)):
                return i
        return None

    def lessRiskyDiscard(self):
        if(anyNoPartial(self)):
            return anyNoPartial(self)
        elif(anyNoFive(self)):
            return anyNoFive(self)
        else:
            return self._selfKnowledge.hand[randint(0, 4)]


    def anyNoInfo(self): #possibilities is only updated with hints
        for i in range(len(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]
            if (len(x.possible["colors"]) == 5 and len(x.possible["numbers"]) == 5):
                return i
        return None


    def decideAction(self):
        #need beliefs of cards
        #beliefs are updated in updateBeliefs. 1 hint of color sets belief of the number
            #to the next number. 1 hint of number, if it matches a color stack and unsure
            #about the color of that card, then set color to where it would be viable to play
        #game.played is array indexed by color of queues? of played cards
        if(self.myImmediatelyPlayableBelief() != None): #any of my cards are playable
            return PlayIntent(indices=[self.myImmediatelyPlayableBelief()])
        elif(len(self.teamMateImmediatelyPlayableUnknown()) > 0 and len(self._board.playedPile )== 0): #teammate's cards are immediately playable, know nothing about, and score = 0
            i = self.teamMateImmediatelyPlayableUnknown()
            return HintIntent(indices = i, feature = self._selfKnowledge.partnerHand[i[0]].number)
        elif(self.activeFireworks() < 4 and self.checkNotPlayedMyCardsFor1(1) != None): #less than or equal to 4 stacks played and you have any 1, play it
            return PlayIntent(indices = self.checkNotPlayedMyCardsFor1(1)[0])
        elif(len(self.altanySureDiscard()) > 0): #no hints left and sure of a discard
            return DiscardIntent(indices=[self.altanySureDiscard(self)])
        elif(self._board.hintTokens < 1 and self.anyNoInfo(self)): #no hints and cards with no info, discard it
            return DiscardIntent(indices = [self.anyNoInfo(self)])
        elif(self._board.hintTokens <= 1): #1 or less hints, do less risky discard -
            return DiscardIntent(indices = [self.lessRiskyDiscard(self)])

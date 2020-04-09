from random import randint
import asyncio
from .implicatureBotSelfKnowledge import implicatureBotSelfKnowledge
from .selfKnowledge import SelfKnowledge
from .intent import PlayIntent, DiscardIntent, HintIntent
from .action import PlaySuccess, PlayFail, Discard, Hint
from .board import Board
from .consts import NUMBER_IN_HAND, HANABOT, AMTS, COLORS
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
        for i in range(len(self.teamMateImmediatelyPlayable())):
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
            if self._board.topPlayedCards()[color] > 0:
                ret.append(color)
        print("number of active fireworks is", len(ret))
        return len(ret)

    def checkNotPlayedMyCardsFor1(self, num):
        lst = self.indicesOfMyNumber(num)
        for x in lst:
            card_colors = self._selfKnowledge.hand[x].possible["colors"] #assumes 0 is removed!
            if(not (len(card_colors) == 1 and (self._board.topPlayedCards())[card_colors[0]] == 1)):
                return x
        return None

    def minNumber(self):
        track = 6
        for c, x in self._board.topPlayedCards().items():
            if x < track:
                track = x
        return track

    def checkDeadByNumber(self, number, pile):
        f = lambda card: card.number == number
        new_pile = list(filter(f, pile))
        return len(new_pile) == AMTS[number + 1]

    def deadFireworkOfColor(self, color):
        ret = 6
        f = lambda card: card.color == color
        discardOfColor = list(filter(f, self._board.discardPile))
        for i in range(5):
            if(self.checkDeadByNumber(i, discardOfColor)):
                ret = i
        return ret

    def altanySureDiscard(self):
        checkedDup = []
        for i in range(len(self._selfKnowledge._hand)):
            x = self._selfKnowledge._hand[i]
            #print("color beliefs for ", i, x.beliefs['colors'])
            #print("number beliefs for ", i, x.beliefs['numbers'])
            if (len(x.beliefs["colors"]) == 1 and len(x.beliefs["numbers"]) == 1  \
            and x.beliefs["numbers"][0] <= self._board.topPlayedCards()[x.beliefs['colors'][0]]):
                return i
            elif(len(x.beliefs["numbers"]) == 1 and x.beliefs["numbers"][0] < self.minNumber()):
                return i
            elif(len(x.beliefs["colors"]) == 1 and self._board.topPlayedCards()[x.beliefs["colors"][0]] == 5):
                return i
            elif(len(x.beliefs["colors"]) == 1 and min(x.beliefs["numbers"]) > self.deadFireworkOfColor(x.beliefs["colors"][0])):
                return i
            elif(len(x.beliefs["colors"]) == 1 and len(x.beliefs["numbers"]) == 1):
                if (x.beliefs["colors"][0], x.beliefs["numbers"][0]) in checkedDup:
                    return i
                else:
                    checkedDup.append((x.beliefs["colors"][0], x.beliefs["numbers"][0]))
        return None

    def anyNoDead(self): #return something that won't cause a dead card
        for i in range(len(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]
            for k in COLORS:
                if len(x.possible["numbers"]) == 1:
                    f = lambda card: card.color == k and card.number == x.possible['numbers'][0]
                    if len(list(filter(f, self._board._discardPile))) < AMTS[x.possible['numbers'][0]] - 1:
                        return i
                else:
                    return i
        return None

    def anyNoInfo(self): #possibilities is only updated with hints
        ret = None
        handAge = 0

        for i in range(len(self._selfKnowledge.hand)):
            x = self._selfKnowledge.hand[i]

            if (len(x.possible["colors"]) > 1 and len(x.possible["numbers"]) > 1):
                if(x.handAge >= handAge):
                    handAge = x.handAge
                    ret = i

        return ret

    def lessRiskyDiscard(self):
        #import pdb; pdb.set_trace();

        if(self.anyNoInfo() != None): #discard oldest card with no information
            return self.anyNoInfo()
        elif(self.anyNoDead() != None): #discard something that will not create dead firework
            return self.anyNoDead()
        else:
            return randint(0, 4)

    def singleHint(self):
        playableUnknowns = self.teamMateImmediatelyPlayableUnknown()
        for x in playableUnknowns:
            f_color = lambda card: card.color == self._selfKnowledge.partnerHand()[i].color
            f_number = lambda card: card.number == self._selfKnowledge.partnerHand()[i].number
            filter_color = list(filter(f_color, playableUnknowns))
            filter_number = list(filter(f_number, playableUnknowns))
            if len(filter_color) == 1:
                return [x, self._selfKnowledge.partnerHand()[i].color()]
            elif len(filter_number) == 1:
                return [x, self._selfKnowledge.partnerHand()[i].number()]
        return None

    def fiveHint(self):
        partnerPerspectiveHand = self._selfKnoweldge._partnerKnowledge.hand()
        for i in range(partnerPerspectiveHand):
            if self.selfKnowledge.partnerHand()[i].number == 5  \
                and len(partnerPerspectiveHand[i].possible() > 1):
                return [i, 5]
        return None

    def unambiguousHint(self):
        return None


    def implicatureHint(self):
        if(self.singleHint() != None):
            return self.singleHint()
        elif(self.fiveHint() != None):
            return self.fiveHint()
        elif(self.unambiguousHint() != None):
            return self.unambiguousHint()
        return None



    def decideAction(self):

        #print("numbers for 1", self._selfKnowledge._hand[1].beliefs["numbers"][0])
        #print("colors as list", self._selfKnowledge._hand[1].beliefs["colors"])
        #print("colors for 1", self._selfKnowledge._hand[1].beliefs["colors"][0])
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
            return PlayIntent(indices = [self.checkNotPlayedMyCardsFor1(1)])
        elif(self.altanySureDiscard() != None): #no hints left and sure of a discard
            #Case 1, believe the number and color and number <= top played number of that color
            #Case 2, believe only number and number is less than any top played card
            #Case 3, believe only the color and the color is already completed
            #Case 4, believe only the color and believe number is greater than dead firework number
            #Case 5, believe color and number and duplicate
            return DiscardIntent(indices=[self.altanySureDiscard()])
        elif(self._board.hintTokens < 1 and self.anyNoInfo() != None): #no hints and cards with no info, discard it
            return DiscardIntent(indices = [self.anyNoInfo()])
        elif(self._board.hintTokens <= 1): #1 or less hints, do less risky discard -
            return DiscardIntent(indices = [self.lessRiskyDiscard()])
        else:
            return None
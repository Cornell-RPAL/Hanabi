from .card import Card, isValidColor, isValidNumber
from .consts import NUMBERS, COLORS, AMTS

from collections import Counter

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for _ in range(AMTS[n])]


class UnknownCard():
    """
    A card in a hand.
    """

    def __init__(self):
        #self._possibleCards = Counter(ALL_CARDS)
        self._handAge = 0
        self._possible = {"colors":COLORS, "numbers":NUMBERS}
        self._beliefs = {"colors":COLORS, "numbers":NUMBERS}

    @property
    def possible(self):
        return self._possible

    @property
    def beliefs(self):
        return self._beliefs
    @property
    def possible_cards(self):
        return self._possibleCards

    @property
    def handAge(self):
        return self._handAge

    def filter(self, f):
        self._possibleCards = Counter(dict([pair
                                            for pair in self._possibleCards.items() if f(pair[0])]))

    def filterPossibiliitiesAndBeliefs(self, feature):
        if(isValidColor(feature)):
            f = lambda color: color == feature
            self._possible["colors"] = list(filter(f, self._possible["colors"]))
            self._beliefs["colors"] = list(filter(f, self._beliefs["colors"]))
        elif(isValidNumber(feature)):
            f = lambda number: number == feature
            self._possible["numbers"] = list(filter(f, self._possible["numbers"]))
            self._beliefs["numbers"] = list(filter(f, self._beliefs["numbers"]))
            #print("possible numbers after filter ", self._possible["numbers"])
            #print("believed numbers after filter", self._beliefs["numbers"])


    def filterBeliefsForImplicature(self, feature, nextCardInColor = None, color = None):
        if(isValidColor(feature) and len(self._beliefs["numbers"]) > 1):
            self._beliefs["numbers"] = [nextCardInColor]
        elif(isValidNumber(feature) and len(self._beliefs["colors"]) > 1):
            self._beliefs["colors"] = [color]
            #print("believed number after implicature filter", self._beliefs["numbers"])


    def setDraw(self, drawSet):
        self._possibleCards = drawSet

    def exclude(self, card):
        assert isinstance(card, Card)
        assert self._possibleCards[card]

        self._possibleCards[card] -= 1

    def updateAge(self):
        self._handAge += 1

    def __str__(self):
        return '!!!!!Unknown card that could be ' + str(self._possibleCards) + '!!!!!'

    def __eq__(self, other):
        possibleCards1 = self._possibleCards.items()
        possibleCards2 = other.possible_cards.items()
        return (possibleCards1.__len__==1 and possibleCards2.__len__ ==1
            and possibleCards1[0][0] == possibleCards2[0][0])

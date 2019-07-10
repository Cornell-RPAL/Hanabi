from card import Card
from game import ALL_CARDS
from collections import Counter


class UnknownCard():
    """
    A card in a hand.
    """

    def __init__(self):
        self._possibleCards = Counter(ALL_CARDS)
        self._handAge = 0

    @property
    def possible_cards(self):
        return self._possibleCards

    @property
    def handAge(self):
        return self._handAge

    def filter(self, f):
        self._possibleCards = Counter(dict([pair
                                            for pair in self._possibleCards.items() if f(pair[0])]))

    def setDraw(self, drawSet):
        self._possibleCards = drawSet
    # def updateFeature(self, feature, applies=True):
    #     """
    #       Removes cards from possible cards
    #       feature: []
    #     """
    #     assert(Card.isValidColor(feature) or Card.isValidNumber(feature))
    #     result = []

    #     def nxor(b1, b2):
    #         return (b1 and b2) or (not(b1) and not(b2))

    #     for card in self.possible_cards:
    #         if nxor(applies, card.color == feature or card.number == feature):
    #             result.append(card)

    #     self.possible_cards = result

    def exclude(self, card):
        assert isinstance(card, Card)
        assert self._possibleCards[card]

        self._possibleCards[card] -= 1

    def updateAge(self):
        self._handAge += 1

    def __str__(self):
        return '[Unknown card that could be ' + str(self._possibleCards) + ']'

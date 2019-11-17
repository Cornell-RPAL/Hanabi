from consts import COLORS, NUMBERS


class Card:
    """
    A card in the Hanabi game.
    """

    def __init__(self, color, number, card_id=-1):
        assert isValidColor(color)
        assert isValidNumber(number)

        self._color = color
        self._number = number
        self._id = card_id

    @property
    def id(self):
        return self._id

    @property
    def color(self):
        return self._color

    @property
    def number(self):
        return self._number

    def hasFeature(self, feature):
        return self.color == feature or self.number == feature

    def __repr__(self):
        return 'Card(#' + str(self._id)+ ') : ' + self.color + ' ' + str(self.number)

    def __eq__(self, other):
        return (self.color == other.color) and (self.number == other.number)

    def same_id(self, other):
        return self.id == other.id

    def __hash__(self):
        return hash(str(self))


def isValidColor(color):
    return color in COLORS

def isValidNumber(number):
    return number in NUMBERS

from consts import COLORS, NUMBERS


class Card:
    """
    A card in the Hanabi game.
    """

    def __init__(self, color, number):
        assert isValidColor(color)
        assert isValidNumber(number)

        self._color = color
        self._number = number

    @property
    def color(self):
        return self._color

    @property
    def number(self):
        return self._number

    def __repr__(self):
        return "Card: " + self.color + " " + str(self.number)

    def __eq__(self, other):
        return (self.color == other.color) and (self.number == other.number)

    def __hash__(self):
        return hash(str(self))

def isValidColor(color):
    return color in COLORS

def isValidNumber(number):
    return number in NUMBERS

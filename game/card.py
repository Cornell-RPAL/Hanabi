from consts import COLORS, NUMBERS


class Card:
    """
    A card in the Hanabi game.
    """

    def __init__(self, color, number):
        assert Card.is_valid_color(color)
        assert Card.is_valid_number(number)

        self.color = color
        self.number = number

    def __repr__(self):
        return "Card: " + self.color + " " + str(self.number)

    def __eq__(self, other):
        return (self.color == other.color) and (self.number == other.number)

    def __hash__(self):
        return hash(str(self))

    @staticmethod
    def is_valid_color(color):
        return color in COLORS

    @staticmethod
    def is_valid_number(number):
        return number in NUMBERS
from consts import COLORS, NUMBERS


class Card:
    """
    A card in the Hanabi game.
    """

    def __init__(self, color, number):
        assert self.is_valid_color(color)
        assert self.is_valid_number(number)

        self.color = color
        self.number = number

    def __repr__(self):
        return "Card: " + self.color + " " + str(self.number)

    def is_valid_color(self, color):
        return color in COLORS

    def is_valid_number(self, number):
        return number in NUMBERS

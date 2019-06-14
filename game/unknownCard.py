from card import Card


class UnknownCard():
    """
    A card in a hand.
    """

    def __init__(self, possible_cards):
        self._possible_cards = possible_cards
        self._hand_age = 0

    @property
    def possible_cards(self):


    def updateFeature(self, feature, applies=True):
        """
          Removes cards from possible cards
          feature: []
        """
        assert(Card.is_valid_color(feature) or Card.is_valid_number(feature))
        result = []

        def nxor(b1, b2):
            return (b1 and b2) or (not(b1) and not(b2))

        for card in self.possible_cards:
            if nxor(applies, card.color == feature or card.number == feature):
                result.append(card)

        self.possible_cards = result

    def exclude(self, card):
        assert isinstance(card, Card)
        assert card in self.possible_cards

        self.possible_cards.remove(card)
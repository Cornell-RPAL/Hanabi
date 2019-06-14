from consts import NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS, COLORS, \
    NUMBERS, AMTS, NUMBER_IN_HAND
from card import Card
from collections import Counter
from random import shuffle

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for i in range(AMTS[n])]


class Game:
    """
    The primary game state for a game of Hanabi.
    """

    def __init__(self):
        self.hint_tokens = NUMBER_OF_HINT_TOKENS
        self.error_tokens = NUMBER_OF_ERROR_TOKENS

        shuffle(ALL_CARDS)
        self.cards = ALL_CARDS
        self.hands = (self.cards[:NUMBER_IN_HAND - 1],
                      self.cards[NUMBER_IN_HAND: NUMBER_IN_HAND * 2 - 1])

        self.discard_pile = []
        self.draw_pile = self.cards[NUMBER_IN_HAND * 2 + 1:]

        self.played_cards = dict(zip(COLORS, [0]*5))

        self.turn = 0
        self.msg = ""

        # if __debug__:

        #     self.check_invariant()

    def hint_to(self, player, feature):
        """
          [player] hints to teammate about [feature].

          Returns:
            list of indices of card that have [feature]

          Args:
            player: integer, 0 or 1
            feature: a valid feature about color or number

          Raises:
            InvalidHint if the hinted feature does not apply to any card in
            the opponent's hand.
        """
        assert player in [0, 1]
        assert self.error_tokens > 0

        class InvalidHint(Exception):
            pass

        if Card.is_valid_color(feature):
            index_list = [i for (i, card) in enumerate(
                self.hands[player]) if card.color == feature]

        if Card.is_valid_number(feature):
            index_list = [i for (i, card) in enumerate(
                self.hands[player]) if card.number == feature]

        if index_list:
            return index_list
        else:
            raise InvalidHint

        self.turn += 1
        self.error_tokens -= 1

    def play_card(self, player, card_ix):
        """
            [player] plays their card at [card_ix]

            Updates board variables according to rules

            Returns:
                True iff card was successfully played on board

        """
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]
        assert self.error_tokens > 0

        success = True

        card = self.hands[player][card_ix]

        # Checks if previous number of same color is on top of color's stack
        if self.played_cards[card.color] == card.number - 1:
            self.played_cards[card.color] += 1
        else:
            success = False
            self.discard_pile.append(card)
            self.error_tokens -= 1

        # Update hand, draw pile
        if len(self.draw_pile):
            self.hands[player][card_ix] = self.draw_pile.pop()
        else:
            self.hands[player][card_ix] = None

        self.turn += 1
        self.error_tokens -= 1

        if __debug__:
            self.check_invariant()
        return success

    def discard(self, player, card_ix):
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]

        card = self.hands[player][card_ix]

        self.discard_pile.append(card)

        if len(self.draw_pile):
            self.hands[player][card_ix] = self.draw_pile.pop()
        else:
            self.hands[player][card_ix] = None

        self.error_tokens += 1

    def check_invariant(self):

        cards_in_game = []

        for color, top_n in self.played_cards.items():
            for n in range(1, top_n):
                cards_in_game.append(Card(color, n))

        cards_in_game += (self.discard_pile + self.draw_pile + self.hands[0] +
                          self.hands[1])

        print(self.hands)
        print('\n')
        print(Counter(cards_in_game).most_common())
        print('\n')
        print(Counter(ALL_CARDS).most_common())
        print(Counter(cards_in_game).most_common()
              == Counter(ALL_CARDS).most_common())

        assert (Counter(cards_in_game).most_common()
                == Counter(ALL_CARDS).most_common())

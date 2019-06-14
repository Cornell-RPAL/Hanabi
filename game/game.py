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
        self._hint_tokens = NUMBER_OF_HINT_TOKENS
        self._error_tokens = NUMBER_OF_ERROR_TOKENS

        shuffle(ALL_CARDS)
        self._cards = ALL_CARDS
        self._hands = (self._cards[:NUMBER_IN_HAND],
                      self._cards[NUMBER_IN_HAND: NUMBER_IN_HAND * 2])

        self._discard_pile = []
        self._draw_pile = self._cards[NUMBER_IN_HAND * 2:]

        #self._played_cards = dict(zip(COLORS, [0]*5))
        self._played_pile = []

        self._turn = 0
        self._msg = ""

        if __debug__:
            self._check_invariant()

    @property
    def turn(self):
        return self._turn

    @property
    def message(self):
        return self._msg

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
        assert self._error_tokens > 0

        class InvalidHint(Exception):
            pass

        if Card.is_valid_color(feature):
            index_list = [i for (i, card) in enumerate(
                self._hands[player]) if card.color == feature]

        if Card.is_valid_number(feature):
            index_list = [i for (i, card) in enumerate(
                self._hands[player]) if card.number == feature]

        if index_list:
            return index_list
        else:
            raise InvalidHint

        self._turn += 1
        self._error_tokens -= 1

    def play_card(self, player, card_ix):
        """
            [player] plays their card at [card_ix]

            Updates board variables according to rules

            Returns:
                True iff card was successfully played on board
        """
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]
        assert self._error_tokens > 0

        success = True

        card = self._hands[player][card_ix]

        # Checks if previous number of same color is on top of color's stack
        # if self._played_cards[card.color] == card.number - 1:
        #    self._played_cards[card.color] += 1

        if (card.number == 1 or
            Card(card.color, card.number - 1) in self._played_pile):
            self._played_pile.append(card)
        else:
            success = False
            self._discard_pile.append(card)
            self._error_tokens -= 1

        # Update hand, draw pile
        if len(self._draw_pile):
            self._hands[player][card_ix] = self._draw_pile.pop()
        else:
            self._hands[player][card_ix] = None

        self._turn += 1
        self._error_tokens -= 1

        if __debug__:
            self._check_invariant()
        return success

    def discard(self, player, card_ix):
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]

        card = self._hands[player][card_ix]

        self._discard_pile.append(card)

        if len(self._draw_pile):
            self._hands[player][card_ix] = self._draw_pile.pop()
        else:
            self._hands[player][card_ix] = None

        self._error_tokens += 1

        if __debug__:
            self._check_invariant()

    def _check_invariant(self):

        cards_in_game = (self._played_pile + self._discard_pile + self._draw_pile
            + self._hands[0] + self._hands[1])

        # for color, top_n in self._played_cards.items():
        #     for n in range(1, top_n):
        #         cards_in_game.append(Card(color, n))

        # cards_in_game += (self._discard_pile + self._draw_pile + self._hands[0] +
        #                  self._hands[1])

        # print(self._hands)
        # print('\n')
        # print(Counter(cards_in_game).most_common())
        # print('\n')
        # print(Counter(ALL_CARDS).most_common())
        # print(Counter(cards_in_game).most_common()
        #       == Counter(ALL_CARDS).most_common())
        # print('\nDiff:\n')
        # print(Counter(ALL_CARDS) - Counter(cards_in_game))

        assert Counter(cards_in_game) == Counter(ALL_CARDS), \
            ('The invariant is broken: cards missing from deck:' + 
            str(Counter(ALL_CARDS) - Counter(cards_in_game)))



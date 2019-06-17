from consts import (
    NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS,
    COLORS, NUMBERS, AMTS, NUMBER_IN_HAND,
    STATE_ACTIVE, STATE_LAST_ROUND, STATE_COMPLETE
)
from card import Card
from collections import Counter
from random import shuffle

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for _ in range(AMTS[n])]


class Game:
    """
    The primary game state for a game of Hanabi.
    """

    def __init__(self):
        self._hintTokens = NUMBER_OF_HINT_TOKENS
        self._errorTokens = NUMBER_OF_ERROR_TOKENS

        shuffle(ALL_CARDS)
        self._cards = ALL_CARDS
        self._hands = (self._cards[:NUMBER_IN_HAND],
                       self._cards[NUMBER_IN_HAND: NUMBER_IN_HAND * 2])

        self._discardPile = []
        self._drawPile = self._cards[NUMBER_IN_HAND * 2:]

        # self._played_cards = dict(zip(COLORS, [0]*5))
        self._playedPile = []

        self._turn = 0
        self._score = 0
        self._msg = ""

        self._lastTurn = 0

        if __debug__:
            self._checkInvariant()

    @property
    def turn(self):
        return self._turn

    @property
    def score(self):
        return len(self._playedPile)

    @property
    def message(self):
        return self._msg

    def hintTo(self, player, feature):
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
        assert self._errorTokens > 0

        class InvalidHint(Exception):
            pass

        if Card.isValidColor(feature):
            index_list = [i for (i, card) in enumerate(self._hands[player])
                          if card.color == feature]

        if Card.isValidNumber(feature):
            index_list = [i for (i, card) in enumerate(self._hands[player])
                          if card.number == feature]

        if index_list:
            return index_list
        else:
            raise InvalidHint

        self._turn += 1
        self._errorTokens -= 1

    def playCard(self, player, card_ix):
        """
            [player] plays their card at [card_ix]

            Updates board variables according to rules

            Returns:
                True iff card was successfully played on board
        """
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]
        assert self._errorTokens > 0

        success = True

        card = self._hands[player][card_ix]

        # Checks if previous number of same color is on top of color's stack
        if (card.number == 1 or
            (Card(card.color, card.number) not in self._playedPile and
             Card(card.color, card.number - 1) in self._playedPile)):
            self._playedPile.append(card)
        else:
            success = False
            self._discardPile.append(card)
            self._errorTokens -= 1

        # Update hand, draw pile
        if len(self._drawPile):
            self._hands[player][card_ix] = self._drawPile.pop()
        else:
            self._hands[player][card_ix] = None

        self._errorTokens -= 1
        self._turn += 1

        if __debug__:
            self._checkInvariant()
        return success

    def discard(self, player, card_ix):
        assert card_ix in range(NUMBER_IN_HAND)
        assert player in [0, 1]

        card = self._hands[player][card_ix]

        self._discardPile.append(card)

        if len(self._drawPile):
            self._hands[player][card_ix] = self._drawPile.pop()
        else:
            self._hands[player][card_ix] = None

        self._errorTokens -= 1
        self._hintTokens += 1
        self._turn += 1

        if __debug__:
            self._checkInvariant()

    def checkState(self):
        if self._errorTokens == 0:
            self._msg = "You made 3 mistakes! There has been an explosion"
            self._state = STATE_COMPLETE
        elif self.score == 25:
            self._msg = "Legendary! You made all five fireworks!"
            self._state = STATE_COMPLETE
        elif self._drawPile == []:
            self._msg = "All cards are gone! You two each have one turn left"
            self._state = STATE_LAST_ROUND
            self._lastTurn = self._turn + 2
        elif (self._state == STATE_LAST_ROUND and
                self._turn == self._lastTurn):
            self._msg = "You have used up all the rounds! Game complete"
            self._state = STATE_COMPLETE

    def _checkInvariant(self):

        cards_in_game = (self._playedPile + self._discardPile + self._drawPile
                         + self._hands[0] + self._hands[1])

        # for color, top_n in self._played_cards.items():
        #     for n in range(1, top_n):
        #         cards_in_game.append(Card(color, n))

        # cards_in_game += (self._discard_pile + self._draw_pile +
        #                   self._hands[0] + self._hands[1])

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

from consts import (
    NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS,
    COLORS, NUMBERS, AMTS, NUMBER_IN_HAND,
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)
from card import Card
from collections import Counter
from random import shuffle

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for _ in range(AMTS[n])]

class InvalidHint(Exception):
    pass
class IncorrectArgumentNumber(Exception):
    pass
class InvalidCommand(Exception):
    pass

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
        self._message = ""
        self._state = STATE_ACTIVE

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
        return self._message
    
    @property
    def hintTokens(self):
        return self._hintTokens
    
    @property
    def errorTokens(self):
        return self._errorTokens
    
    @property
    def hands(self):
        return self._hands

    @property
    def state(self):
        return self._state

    def topPlayedCards(self):
        """
        A dictionary representing the top card of each color. 
        
        The value is [0] if there is no cards of the color.
        """
        d = dict(zip(COLORS, [0]*5))
        for c in self._playedPile:
            if d[c.color] < c.number:
                d[c.color] = c.number
        return d

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

        index_list = []

        if Card.isValidColor(feature):
            index_list = [i for (i, card) in enumerate(self._hands[player])
                          if card.color == feature]

        if Card.isValidNumber(feature):
            index_list = [i for (i, card) in enumerate(self._hands[player])
                          if card.number == feature]

        if index_list:
            self._turn += 1
            self._hintTokens -= 1
            return index_list
        else:
            raise InvalidHint


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
        if (Card(card.color, card.number) not in self._playedPile and
             (card.number == 1 or Card(card.color, card.number - 1) in self._playedPile)):
            self._playedPile.append(card)
            if card.number == 5:
                self._hintTokens += 1
        else:
            success = False
            self._discardPile.append(card)
            self._errorTokens -= 1
        


        # Update hand, draw pile
        if len(self._drawPile):
            self._hands[player][card_ix] = self._drawPile.pop()
        else:
            self._hands[player][card_ix] = None

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

    def _checkState(self):
        if self._errorTokens == 0:
            self._message = "You made 3 mistakes! There has been an explosion\n"
            self._state = STATE_COMPLETE
        elif self.score == 25:
            self._message = "Legendary! You made all five fireworks!\n"
            self._state = STATE_COMPLETE
        elif self._drawPile == []:
            self._message = "All cards are gone! \
                You two each have one turn left\n"
            self._state = STATE_LAST_ROUND
            self._lastTurn = self._turn + 2
        elif (self._state == STATE_LAST_ROUND and
                self._turn == self._lastTurn):
            self._message = "You have used up all the rounds! Game complete\n"
            self._state = STATE_COMPLETE

    def update(self, player, command):
        wl = command.split(' ')
        verb = wl[0]
        args = wl[1:]

        try:
            if verb == "help":
                self._message = 'You can either "play [card index]",\
                    "discard [card index]", or "hint [number or color]"\n'
            else:
                if len(args) != 1:
                    raise IncorrectArgumentNumber
                elif verb not in ["play", "discard", "hint"]:
                    raise InvalidCommand
                else:
                    if verb == "play":
                        self._message = "Your partner played " + \
                            str(
                                self._hands[player][int(args[0])]
                                )
                        self.playCard(player, int(args[0]))
                    elif verb == "discard":
                        self.discard(player, int(args[0]))
                        self._message = "Your partner discarded " + str(args[0])
                    elif verb == "hint":
                        if args[0].isdigit():
                            il = self.hintTo(1 - player, int(args[0]))
                        else:
                            il = self.hintTo(1 - player, args[0])
                        self._message = ("Your partner hinted that your cards " + 
                        "at indices " + str(il) + " are " + str(args[0]))
                    self._state = STATE_CONTINUE
                    self._message += '\n'
                    return True

        except InvalidHint:
            self._message = ("Invalid feature, please enter color, or" +
                "number from 0 to " + str(NUMBER_IN_HAND))
        except IncorrectArgumentNumber:
            self._message = "Invalid number of arguments, type [help] \
                    for help. \n"
        except InvalidCommand:
            self._message = "Invalid command! Type [help] for help."
        self._message += '\n'
        self._checkState()
        return False

    def _checkInvariant(self):

        cards_in_game = (self._playedPile + self._discardPile + self._drawPile
                         + self._hands[0] + self._hands[1])

        assert Counter(cards_in_game) == Counter(ALL_CARDS), \
            ('The invariant is broken: cards missing from deck:' +
             str(Counter(ALL_CARDS) - Counter(cards_in_game)))

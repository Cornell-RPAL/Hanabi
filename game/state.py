from consts import *
from card import Card
from unknownCard import UnknownCard
from random import shuffle

ALL_CARDS = [Card(c, n)
             for n in NUMBERS for c in COLORS for i in range(AMTS[n])]


class State:
    """
    The primary game state for a game of Hanabi.
    """

    def __init__(self):
        hint_tokens = NUMBER_OF_HINT_TOKENS
        error_tokens = NUMBER_OF_ERROR_TOKENS

        shuffle(ALL_CARDS)
        cards = ALL_CARDS
        partner_hand = cards[:NUMBER_IN_HAND - 1]

        # the reality of our hand; but not needed
        _own_hand = cards[NUMBER_IN_HAND: NUMBER_IN_HAND * 2]

        discard_pile = []
        draw_pile = cards[NUMBER_IN_HAND * 2 + 1:]

        own_knowledge = [UnknownCard(cards[NUMBER_IN_HAND:])] * 5

        played_cards = dict(zip(COLORS, [0]*5))

        turn = 0

    def update(self, action):

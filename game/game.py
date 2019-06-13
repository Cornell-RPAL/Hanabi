from const import *
from card import Card
from random import shuffle

ALL_CARDS = [Card(c, n) for n in NUMBERS for c in COLORS for i in range(AMTS[n])] 

class Game:
  """
  The primary game state for a game of Hanabi.
  """
  def __init__(self):
    hint_tokens = NUMBER_OF_HINT_TOKENS
    error_tokens = NUMBER_OF_ERROR_TOKENS

    shuffle(ALL_CARDS)
    cards = ALL_CARDS
    hands = (cards[:NUMBER_IN_HAND - 1],
            cards[NUMBER_IN_HAND : NUMBER_IN_HAND * 2])
    
    discard_pile = []
    draw_pile = cards[NUMBER_IN_HAND * 2 + 1:]

    played_cards = dict(zip(COLORS, [0]*5))

    turn = 0
    msg = ""

  def hint_to(self, player, feature):
    """
      [player] hints to teammate about [feature].

      Args:
        player: integer, 0 or 1
        feature: a valid feature about color or number

      Raises:
        InvalidHint if the hinted feature does not apply to any card in the 
        opponent's hand.
    """
    class InvalidHint(Exception):
      pass
    if is_color(feature):
      index_list = [i for (card, i) in enumerate(hands[player]) if card.color == feature]

    if is_number(feature):
      index_list = [i for (card, i) in enumerate(hands[player]) if card.number == feature]
    
    if index_list:
      msg = "Your teammate hinted that your cards at" + str(index_list) +\
        "is " + feature
    else:
      raise InvalidHint

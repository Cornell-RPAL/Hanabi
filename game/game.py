from consts import *
from card import Card
from random import shuffle

ALL_CARDS = [Card(c, n) for n in NUMBERS for c in COLORS for i in range(AMTS[n])] 

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
            self.cards[NUMBER_IN_HAND : NUMBER_IN_HAND * 2])
    
    self.discard_pile = []
    self.draw_pile = self.cards[NUMBER_IN_HAND * 2 + 1:]

    self.played_cards = dict(zip(COLORS, [0]*5))

    self.turn = 0
    self.msg = ""

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
      index_list = [i for (card, i) in enumerate(self.hands[player]) if card.color == feature]

    if is_number(feature):
      index_list = [i for (card, i) in enumerate(self.hands[player]) if card.number == feature]
    
    if index_list:
      self.msg = "Your teammate hinted that your cards at" + str(index_list) +\
        "is " + feature
    else:
      raise InvalidHint

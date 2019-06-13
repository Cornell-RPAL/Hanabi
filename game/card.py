from enum import Enum
from const import *

class Card:
  """
  A card in the Hanabi game.
  """

  def __init__(self, color, number):
    assert is_color(color)
    assert is_number(number)
    
    self.color = color
    self.number = number

  def __repr__(self):
    return "Card: " + self.color + " " + str(self.number)




  
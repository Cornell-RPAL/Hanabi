
### BOARD CONSTANTS
NUMBER_OF_HINT_TOKENS = 8
NUMBER_OF_ERROR_TOKENS = 3

### CARD CONSTANTS 
COLORS = ["red", "yellow", "blue", "green", "white"]
NUMBERS = range(1, 6)
AMTS = {
  1: 3, 
  2: 2,
  3: 2,
  4: 2,
  5: 1
}

NUMBER_IN_HAND = 5


def is_color(color):
  return color in COLORS

def is_number(number):
  return number in NUMBERS

# 
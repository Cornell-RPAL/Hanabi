import curses
from game import Game
from consts import (
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)


class Hanabi():

    def __init__(self):
        self._game = Game()
        self._player = 0
        self._state = STATE_ACTIVE
        self._prevCommand = ""
        self._prevError = ""
        self._output = ""

    @property
    def output(self):
        return self._output

    def update(self, command):
        if self._state in [STATE_ACTIVE, STATE_LAST_ROUND]:
            self._displayGame(self._game, self._player)

            if self._game.update(self._player, command):
                self._prevCommand = self._game.message
                self._prevError = ''
                self._player = 1 - self._player
            else:
                self._prevError = self._game.message
            self._state = self._game.state
            return True

        elif self._state == STATE_CONTINUE:
            self._output = ""
            self._output += ("It is player " + str(self._player) + "'s turn. " +
                          "Press enter to continue.")
            
            self._state = STATE_ACTIVE
            return True

        elif self._state == STATE_COMPLETE:
            self._output = ""
            self._output += (self._game.message)
            self._output += ('Your final score is ' + self._game.score)
            
            return False

    def _displayGame(self, g, player):
        self._output += ("Here is all the information. ")
        self._output += (self._prevCommand)
        self._displayBoard(g, player)
        self._output += (self._prevError)
        self._output += ("Now say what you want to do.")

    def _displayBoard(self, g, player):
        p_hand = ' '.join(['[' + card.color + ' ' + str(card.number) + ']'
                           for card in g.getHand(1 - player)])
        played = ' '.join(['[' + color + ' ' + str(number) + ']'
                           for color, number in g.topPlayedCards().items()])

        self._output += ("Hint Tokens: " + str(g.board.hintTokens) + '\n')
        self._output += ("Error Tokens: " + str(g.board.errorTokens) + '\n')
        self._output += ("Turn: " + str(g.turn) + '\n\n')
        self._output += ("Partner Hand: " + p_hand + '\n\n')
        self._output += ("Played Cards: " + played + '\n\n')
        self._output += ("Number left in draw pile: " + 
            str(len(g.board.drawPile)) + '\n\n')

        if (g.message != self._prevCommand and
                g.message != self._prevError):
            self._output += (g.message + '\n')
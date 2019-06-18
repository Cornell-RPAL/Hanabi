import curses
import time
from game import Game
from consts import (
    NUMBER_OF_HINT_TOKENS, NUMBER_OF_ERROR_TOKENS,
    COLORS, NUMBERS, AMTS, NUMBER_IN_HAND,
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)
stdscr = curses.initscr()

# stdscr.addstr("Hello World!\n")
# stdscr.refresh()
# s = stdscr.getstr()

class Hanabi():

    def start(self):
        curses.echo()
        self._game = Game()
        self._player = 0
        self._state = STATE_ACTIVE
        self._prevCommand = ""
        self._prevError = ""

    def update(self, stdscr):
        if self._state in [STATE_ACTIVE, STATE_LAST_ROUND]:
            self._displayGame(stdscr, self._game, self._player)
            
            command = stdscr.getstr().decode()
            if self._game.update(self._player, command):
                self._prevCommand = self._game.message
                self._prevError = ''
                self._player = 1 - self._player
            else:
                self._prevError = self._game.message
            self._state = self._game.state 
            return True
                
        elif self._state == STATE_CONTINUE:
            stdscr.clear()
            stdscr.addstr("It is player " + str(self._player) + "'s turn. " +
                "Press enter to continue.")
            stdscr.refresh()
            while stdscr.getstr():
                pass
            self._state = STATE_ACTIVE
            return True

        elif self._state == STATE_COMPLETE:
            stdscr.clear()
            stdscr.addstr(self._game.message)
            stdscr.addstr('Your final score is ' + self._game.score)
            stdscr.refresh()
            while stdscr.getstr():
                pass
            return False

        
    def _displayGame(self, scr, g, player):
        stdscr.clear()
        stdscr.addstr("~HANABI~\n\n")
        stdscr.addstr(self._prevCommand)
        self._displayBoard(stdscr, g, player)
        stdscr.addstr(self._prevError, curses.A_BOLD)
        stdscr.addstr(">>> ", curses.A_BLINK)
        stdscr.refresh()

    def _displayBoard(self, scr, g, player):
        p_hand = ' '.join(['[' + card.color + ' ' + str(card.number) + ']' \
            for card in g.hands[1 - player]])
        played = ' '.join(['[' + color + ' ' + str(number) + ']' \
            for color, number in g.topPlayedCards().items()])

        scr.addstr("Hint Tokens: " + str(g.hintTokens) + '\n')
        scr.addstr("Error Tokens: " + str(g.errorTokens) + '\n')
        scr.addstr("Turn: " + str(g.turn) + '\n\n')
        scr.addstr("Partner Hand: " + p_hand + '\n\n')
        scr.addstr("Played Cards: " + played + '\n\n')
        if (g.message != self._prevCommand and
            g.message != self._prevError):
            scr.addstr(g.message + '\n')
        




hanabi = Hanabi()

def main(scr):
    hanabi.start()
    while hanabi.update(scr):
        pass

curses.wrapper(main)

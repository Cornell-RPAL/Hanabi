import curses
from game import (Game, InvalidCommand, IncorrectArgumentNumber, InvalidHint,
NoHintTokens, NoErrorTokens)
from consts import (
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)
from consts import NUMBER_IN_HAND
from action import Action, PlayCard, Discard, Hint
from hanabot import Hanabot

stdscr = curses.initscr()

# stdscr.addstr("Hello World!\n")
# stdscr.refresh()
# s = stdscr.getstr()


class Hanabi():

    def __init__(self):
        curses.echo()
        self._game = Game()
        self._player = 0
        self._bot = Hanabot(self._game, 1-self._player)
        self._state = STATE_ACTIVE
        self._prevCommand = ""
        self._prevError = ""
        self._message = ""

    def parseCommand(self, player, command):
        wl = command.split(' ')
        verb = wl[0]
        args = wl[1:]
        self._message = ""

        try:
            if verb == "help":
                stdscr.addstr ('You can either "play [card index]",\
                    "discard [card index]", or "hint [number or color]"\n')
            else:
                if len(args) != 1:
                    raise IncorrectArgumentNumber
                elif verb not in ["play", "discard", "hint"]:
                    raise InvalidCommand
                else:
                    if verb == "play":
                        player_action = PlayCard(self._game, player, card_ix= int(args[0]))
                        player_action.act(self._bot)
                        self._message += (player_action.__str__())
                    elif verb == "discard":
                        player_action = Discard(self._game, player, card_ix= int(args[0]))
                        player_action.act(self._bot)
                        self._message +=(player_action.__str__())
                    elif verb == "hint":
                        if args[0].isdigit():
                            player_action = Hint(self._game, player, feature= int(args[0]))
                        else:
                            player_action = Hint(self._game, player, feature= args[0])
                        player_action.act(self._bot)
                    self._state = STATE_CONTINUE
                    self._message += ('\n')
                    bot_act = self._bot.decideAction()
                    bot_act.act(self._bot)
                    self._message += bot_act.__str__()
                    return True

        except InvalidHint:
            self._message += ("Invalid feature, please enter color, or" +
                             "number from 0 to " + str(NUMBER_IN_HAND))
        except NoHintTokens:
            self._message += ("No more hint tokens, you may only play/discard")
        except NoErrorTokens:
            self._message += ("No more error tokens, the game has ended")
            self._state = STATE_COMPLETE
        except IncorrectArgumentNumber:
            self._message += ("Invalid number of arguments, type [help] \
                    for help. \n")
        except InvalidCommand:
            self._message += ("Invalid command! Type [help] for help.")
        self._message +=('\n')
        self._game._checkState()
        return False

    def update(self, stdscr):
        if self._state in [STATE_ACTIVE, STATE_LAST_ROUND]:
            self._displayGame(stdscr, self._game, self._player)

            command = stdscr.getstr().decode()
            if self.parseCommand(self._player, command):
                self._prevCommand = self._game.message
                self._prevError = ''
                #self._player = 1 - self._player
            else:
                self._prevError = self._game.message
            self._state = self._game.state
            return True

        elif self._state == STATE_CONTINUE:
            stdscr.clear()
            self._message +=("It is player " + str(self._player) + "'s turn. " +
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
        stdscr.addstr(self._message)
        self._displayBoard(stdscr, g, player)
        stdscr.addstr(self._prevError, curses.A_BOLD)
        stdscr.addstr(">>> ", curses.A_BLINK)
        stdscr.refresh()

    def _displayBoard(self, scr, g, player):
        p_hand = ' '.join(['[' + card.color + ' ' + str(card.number) + ']'
                           for card in g.getHand(1 - player)])
        played = ' '.join(['[' + color + ' ' + str(number) + ']'
                           for color, number in g.topPlayedCards().items()])

        scr.addstr("Hint Tokens: " + str(g.board.hintTokens) + '\n')
        scr.addstr("Error Tokens: " + str(g.board.errorTokens) + '\n')
        scr.addstr("Turn: " + str(g.turn) + '\n\n')
        scr.addstr("Partner Hand: " + p_hand + '\n\n')
        scr.addstr("Played Cards: " + played + '\n\n')
        scr.addstr("Number left in draw pile: " + 
            str(len(g.board.drawPile)) + '\n\n')

        
        if (g.message != self._prevCommand and
                g.message != self._prevError):
            scr.addstr(g.message + '\n')


def main(scr):
    hanabi = Hanabi()
    while hanabi.update(scr):
        pass


curses.wrapper(main)

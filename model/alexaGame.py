import curses
from game import Game, InvalidCommand, IncorrectArgumentNumber, InvalidHint, NoHintTokens
from action import Action, PlayCard, Discard, Hint
from hanabot import Hanabot
from consts import (
    STATE_ACTIVE, STATE_CONTINUE, STATE_LAST_ROUND, STATE_COMPLETE
)
from consts import NUMBER_IN_HAND

class Hanabi():

    def __init__(self):
        self._game = Game()
        self._player = 0
        self._bot = Hanabot(self._game, 1-self._player)
        self._state = STATE_ACTIVE
        self._prevCommand = ""
        self._prevError = ""
        self._output = ""

    @property
    def output(self):
        return self._output
    
    def parseCommand(self, player, command):
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
                        player_action = PlayCard(self._game, player, card_ix= int(args[0]))
                        player_action.act()
                        self._message=player_action.__str__()
                    elif verb == "discard":
                        player_action = Discard(self._game, player, card_ix= int(args[0]))
                        player_action.act()
                        self._message=player_action.__str__()
                    elif verb == "hint":
                        if args[0].isdigit():
                            player_action = Hint(self._game, player, feature= int(args[0]))
                        else:
                            player_action = Hint(self._game, player, feature= args[0])
                        il = player_action.act()
                        self._message = ("Your partner hinted that your cards " 
                                         + "at indices " + str(il) + " are " + 
                                         str(args[0]))
                    self._state = STATE_CONTINUE
                    self._message += '\n'
                    self._bot.inform(player_action)
                   # bot_act = self._bot.decideAction(self._game)
                    return True

        except InvalidHint:
            self._message = ("Invalid feature, please enter color, or" +
                             "number from 0 to " + str(NUMBER_IN_HAND))
        except NoHintTokens:
            self._message = ("No more hint tokens, you may only play/discard")
        except IncorrectArgumentNumber:
            self._message = "Invalid number of arguments, type [help] \
                    for help. \n"
        except InvalidCommand:
            self._message = "Invalid command! Type [help] for help."
        self._message += '\n'
        self._game._checkState()
        return False

    def update(self, command):
        if self._state in [STATE_ACTIVE, STATE_LAST_ROUND]:
            self._displayGame(self._game, self._player)

            if self.parseCommand(self._player, command):
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

import curses
import time
from game import Game
from consts import COLORS, NUMBER_IN_HAND
stdscr = curses.initscr()

# stdscr.addstr("Hello World!\n")
# stdscr.refresh()
# s = stdscr.getstr()


def main(stdscr):

    curses.echo()
    g = Game()
    player = 0

    previousTurn = ""

    newTurn = True

    while True:
        stdscr.clear()
        
        if newTurn:
            stdscr.addstr("It is player " + str(player) + "'s turn. " +
            "Press enter to continue.")
        
            while stdscr.getstr():
                pass

            stdscr.addstr("")

        displayGame(stdscr, g, player)

        stdscr.addstr(">>> ", curses.A_BLINK)

        

        
        stdscr.refresh()
        wl = (stdscr.getstr().decode()).split(' ')
        command = wl[0]
        args = wl[1:]
        if command == "help":
            stdscr.addstr('You can either "play [card index]",')
            stdscr.addstr('"discard [card index]", \n')
            stdscr.addstr('or "hint [number or color]"')
            stdscr.refresh()
            time.sleep(3)
        elif len(args) != 1:
            stdscr.addstr("Invalid number of arguments, type [help] \
                for help. \n")
        elif command == "play":
            if not args[0].isdigit() \
                or int(args[0]) not in range(NUMBER_IN_HAND):
                stdscr.addstr("Invalid index, please enter number from \
                    0 to " + str(NUMBER_IN_HAND))
            else:
                g.playCard(player, int(args[0]))
                newTurn = True
                continue
        elif command == "discard":
            if not args[0].isdigit() \
                or int(args[0]) not in range(NUMBER_IN_HAND):
                stdscr.addstr("Invalid index, please enter number from \
                    0 to " + str(NUMBER_IN_HAND))
            else:
                g.discard(player, int(args[0]))
                newTurn = True
                continue
        elif command == "hint":
            if (args[0].isdigit() and int(args[0]) \
                 in range(1, NUMBER_IN_HAND + 1)):
                g.hintTo(1 - player, int(args[0]))
                newTurn = True
                continue
            elif args[0] in COLORS:
                g.hintTo(1 - player, args[0])
                newTurn = True
                continue
            else:
                stdscr.addstr("Invalid feature, please enter color, \
                    or number from 0 to " + str(NUMBER_IN_HAND))
        else:
            stdscr.addstr("Invalid command! Type [help] for help.")
        player = 1 - player
        stdscr.refresh()
        




def displayGame(scr, g, player):
    p_hand = ' '.join(['[' + card.color + ' ' + str(card.number) + ']' \
        for card in g.hands[1 - player]])
    played = ' '.join(['[' + color + ' ' + str(number) + ']' \
        for color, number in g.topPlayedCards().items()])

    scr.addstr("HANABI\n\n")
    scr.addstr("Hint Tokens: " + str(g.hintTokens) + '\n')
    scr.addstr("Error Tokens: " + str(g.errorTokens) + '\n')
    scr.addstr("Turn: " + str(g.turn) + '\n\n')
    scr.addstr("Partner Hand: " + p_hand + '\n\n')
    scr.addstr("Played Cards: " + played + '\n\n')
    scr.addstr(g.message + '\n')
    return








#curses.endwin()
curses.wrapper(main)

import curses
import time
stdscr = curses.initscr()

stdscr.addstr("Hello World!\n")
stdscr.refresh()
s = stdscr.getstr()

curses.endwin()
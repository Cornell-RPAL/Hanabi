import unittest
from model.board import Board
from model.simpleImplicatureBot import simpleImplicatureBot
from model.implicatureBotSelfKnowledge import implicatureBotSelfKnowledge
from model.consts import HANABOT
from model.card import Card

class TestActionSelection(unittest.TestCase):
    def test_move_select(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 1)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(2, [1])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), PlayIntent(indices = [1]))




if __name__ == '__main__':
    unittest.main()

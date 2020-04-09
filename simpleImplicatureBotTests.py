import unittest
from model.board import Board
from model.simpleImplicatureBot import simpleImplicatureBot
from model.implicatureBotSelfKnowledge import implicatureBotSelfKnowledge
from model.consts import HANABOT
from model.card import Card
from model.intent import PlayIntent, DiscardIntent, HintIntent
from model.action import Action, Discard
class TestActionSelection(unittest.TestCase):
    #hand indexed from 0
    def test_move_select(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 1)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(2, [1])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), PlayIntent(indices = [1]))

    def test_interesting_behavior_one(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 1)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(1, [1])
        selfknowledge.updateWithHint("blue", [1])
        bot = simpleImplicatureBot(board, selfknowledge)
        print("interesting behavior 1 action:", bot.decideAction())
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [1]))

    def test_initial_hint_teammate_ones(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        bot = simpleImplicatureBot(board, selfknowledge)
        #print("decided action in initial hint teammate ones ", bot.decideAction())
        self.assertEqual(bot.decideAction(), HintIntent(feature = 1, indices = [0]))

    def test_guess_placing_ones(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 1), Card("red", 1), Card("green", 1)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(1, [1, 2])
        bot = simpleImplicatureBot(board, selfknowledge)
        #print("decided action in guess placing ones is", bot.decideAction())
        self.assertEqual(bot.decideAction(), PlayIntent(indices = [1]))

    def test_any_sure_discard_1(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 1), Card("red", 1), Card("green", 1)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(1, [4])
        selfknowledge.updateWithHint("blue", [4])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [4]))

    def test_any_sure_discard_2(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 2), Card("red", 2), Card("green", 2), Card("white", 2), Card("yellow", 2)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint(1, [3])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [3]))

    def test_any_sure_discard_3(self):
        board = Board([Card("red", 1), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = [Card("blue", 5)]
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("blue", [2])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [2]))

    def test_any_sure_discard_4(self):
        board = Board([Card("red", 1), Card("red", 1), Card("red", 1), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updatePartnerAction(Discard(1 - HANABOT, Card("red", 1), indices=[0]))
        selfknowledge.updatePartnerAction(Discard(1 - HANABOT, Card("red", 1), indices=[1]))
        selfknowledge.updatePartnerAction(Discard(1 - HANABOT, Card("red", 1), indices=[2]))
        selfknowledge.updateWithHint("red", [0])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [0]))

    def test_any_sure_discard_5(self):
        board = Board([Card("red", 2), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("red", [0, 1])
        selfknowledge.updateWithHint(3, [0, 1])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [1]))

    def test_any_no_info(self):
        board = Board([Card("red", 2), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        board._hintTokens = 0
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("red", [0, 2])
        bot = simpleImplicatureBot(board, selfknowledge)
        self.assertEqual(bot.decideAction(), DiscardIntent(indices = [1]))

    def test_less_risky_discard1(self):
        board = Board([Card("red", 2), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        board._hintTokens = 1
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("red", [0, 2])
        bot = simpleImplicatureBot(board, selfknowledge)
        intent = bot.decideAction()
        print(intent)
        self.assertEqual(intent, DiscardIntent(indices = [4]))

    def test_less_risky_discard2(self):
        board = Board([Card("red", 2), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("red", [0])
        selfknowledge.updateWithHint("blue", [1])
        selfknowledge.updateWithHint("green", [2])
        selfknowledge.updateWithHint("white", [3])
        selfknowledge.updateWithHint(5, [0, 1, 2, 3])
        selfknowledge.updateWithHint(3, [4])
        board.hintTokens = 1
        bot = simpleImplicatureBot(board, selfknowledge)
        intent = bot.decideAction()
        self.assertEqual(intent, DiscardIntent(indices = [4]))

    def test_less_risky_discard3(self):
        board = Board([Card("red", 2), Card("red", 2), Card("red", 3), Card("red", 4), Card("red", 5)])
        board.playedPile = []
        selfknowledge = implicatureBotSelfKnowledge(board, HANABOT)
        selfknowledge.updateWithHint("red", [0])
        selfknowledge.updateWithHint("blue", [1])
        selfknowledge.updateWithHint("green", [2])
        selfknowledge.updateWithHint("white", [3])
        selfknowledge.updateWithHint("yellow", [4])
        selfknowledge.updateWithHint(5, [0, 1, 2, 3, 4])

        board.hintTokens = 1;
        bot = simpleImplicatureBot(board, selfknowledge)
        print("LRD3", bot.decideAction())

if __name__ == '__main__':
    unittest.main()

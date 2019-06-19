from game import Game
from selfKnowledge import SelfKnowledge

g = Game()
sk = SelfKnowledge(g, 0)
sk.updateWithHint("red", [0,1])
sk.updateWithHint(1, [0,2])
print(str(sk))

# g.hintTo(0, "red")
# g.playCard(0, 0)
# g.playCard(0, 1)
# g.discard(1, 2)


class Action:
    """
    An interface that represents the action of a player.
    """
    def __init__ (self, player, board, feature=None, indices=[]):
        """
        [feature] is [None] iff action is [Play] or [Discard].
        """
        self._player = player
        self._board = board
        self._indices = indices
        self._feature = feature

    def __str__ (self):
        return ""

class AttemptPlay(Action):
    def act(self, bot):
        # baxter.attemptPlay()
        # await sensoryBuffer.cardInGripper
        if cardInGripper.playable():
            # baxter.continuePlay()
            bot.inform(Play(self._player, self._board))
        else:
            # baxter.continueDiscard()
            bot.inform(Fail(self._player, self._board))
            

class Play(Action):

    def act(self, game, bot):
        bot.inform(self)
        if self._player == HANABOT:
            # TODO: baxter

    def __str__(self):
        return " play card at index " + str(self.card)+"\n"

class Discard(Action):

    def act(self, game, bot):
        game.discard (player = self._player, card = self.card)
        bot.inform(self)

    def __str__(self):
        return " discard card at index " + str(self.card) +"\n"

class Hint(Action):

    def act(self, game, bot):
        il = game.hintTo(player = 1-self._player, feature = self.feature)
        self.il = il
        bot.informHint(self, il)

    def __str__(self):
        idx = ""
        for num in self.il:
            idx += " "+ str(num)
        return "Hint: your cards at indices "+ idx + " are "+ str(self.feature) +"\n"

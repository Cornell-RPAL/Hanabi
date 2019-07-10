from game import Game
#from hanabot import Hanabot


class Action:
    """
    Represents the action of a player.
    """
    def __init__ (self, game, player_num, card_ix=None, feature=None):
        self.game = game
        self.player_num = player_num
        self.card_ix = card_ix
        self.il = None
        self.feature = feature

    def act(self, bot):
        raise NotImplementedError
    
    def __str__ (self):
        return ""

class PlayCard(Action):

    def act(self, bot):
        if self.game.playCard (player= self.player_num, card_ix=self.card_ix):
            bot.inform(self)
        else: bot.inform(Discard(self.game, self.player_num, card_ix=self.card_ix))

    def __str__(self):
        return "Your partner played their card at index " + str(self.card_ix)+"\n"

class Discard(Action):

    def act(self, bot):
        self.game.discard (player = self.player_num, card_ix = self.card_ix)
        bot.inform(self)

    def __str__(self):
        return "Your partner discarded their card at index " + str(self.card_ix) +"\n"

class Hint(Action):

    def act(self, bot):
        il = self.game.hintTo(player = 1-self.player_num, feature = self.feature)
        self.il = il
        bot.inform(self, il)

    def __str__(self):
        return str(self.il) + " are "+ str(self.feature) +"\n"

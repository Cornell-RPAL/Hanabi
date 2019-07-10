from game import Game

class Action:
    """
    Represents the action of a player.
    """
    def __init__ (self, player_num, card_ix=None, feature=None):
        self.player_num = player_num
        self.card_ix = card_ix
        self.il = None
        self.feature = feature

    def act(self, game, bot):
        raise NotImplementedError
    
    def __str__ (self):
        return ""

class PlayCard(Action):

    def act(self, game, bot):
        if game.playCard (player= self.player_num, card_ix=self.card_ix):
            bot.inform(self)
        else: bot.inform(Discard(game, self.player_num, self.card_ix))

    def __str__(self):
        return " play card at index " + str(self.card_ix)+"\n"

class Discard(Action):

    def act(self, game, bot):
        game.discard (player = self.player_num, card_ix = self.card_ix)
        bot.inform(self)

    def __str__(self):
        return " discard card at index " + str(self.card_ix) +"\n"

class Hint(Action):

    def act(self, game, bot):
        il = game.hintTo(player = 1-self.player_num, feature = self.feature)
        self.il = il
        bot.informHint(self, il)

    def __str__(self):
        idx = ""
        for num in self.il:
            idx += " "+ str(num)
        return "Hint: your cards at indices "+ idx + " are "+ str(self.feature) +"\n"

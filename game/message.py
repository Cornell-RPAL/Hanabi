

class Message():

    def __init__(self, game):
        self._game = game

    def startMessage(self):
        return "Welcome to Hanabi. I'm ready to play a game."

    def actionMessage(self, action):
        return "I would "+ str(action)
    
    def respond(self, action):
        return actionMessage(action)
    
class TerminalText(Message):

    def startMessage(self):

    def hintMessage(self, action):
    
    def playMessge(self, action):

    def discardMessage(self, action):

    def respond(self):

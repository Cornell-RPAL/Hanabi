

class Message():

    def __init__(self, game):
        self._game = game

    def startMessage(self):
        return "Welcome to Hanabi!"

    def hintMessage(self, action):
        return "Hint: your card at indices " + str(action) 
    
    def playMessge(self, action):
        return ''

    def discardMessage(self, action):
        return ''
    
    def respond(self):
        return ''
    
class TerminalText(Message):

    def startMessage(self):

    def hintMessage(self, action):
    
    def playMessge(self, action):

    def discardMessage(self, action):

    def respond(self):



class Message():

    def __init__(self):
        pass

    def startMessage(self):
        return "Welcome to Hanabi. I'm ready to play a game."

    def actionMessage(self, intent):
        return "I would "+ str(intent)

    def respond(self, action):
        return self.actionMessage(action)

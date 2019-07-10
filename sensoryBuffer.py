from game.action import Action

class SensoryBuffer():
    def __init__(self):
        self._text = "a"

    def setText(self, text):
        print("Set in a buffer: " + text)
        self._text = text

    @property
    def text(self):
        return self._text

    def textToAction(self):
        


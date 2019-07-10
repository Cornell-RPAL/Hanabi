from game.action import Action

class SensoryBuffer():
    def __init__(self, frame_num=30):
        self._text = "a"
        self._frames = []
        self._frame_num = frame_num

    def setText(self, text):
        print("Set in a buffer: " + text)
        self._text = text

    @property
    def text(self):
        return self._text

    def getFrame(self):
        return self._frames.pop()

    def addFrame(self, frame):
        if len(self._frames()) >= frame_num:
            self._frames.pop(0)
        self._frames.append(frame)

    def textToAction(self):
        pass


class SensoryBuffer():
    def __init__(self):
        self._text = "a"

    def setText(self, text):
        self._text = text

    @property
    def text(self):
        return self._text
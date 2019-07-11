class OutputBuffer():
    def __init__(self):
        self._text = ''
        self._action = None

    def setText(self, text):
        self._text = text

    @property
    def text(self):
        return self._text

    @property
    def action(self):
        return self._action
    
    @action.setter
    def action(self, a):
        self._action = a

    

class OutputBuffer():
    def __init__(self):
        self._text = ''
        self._action = None

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, text):
        self._text = text
    
    @property
    def action(self):
        return self._action
    
    @action.setter
    def action(self, a):
        self._action = a
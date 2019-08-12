class OutputBuffer():
    def __init__(self):
        self._text = ''
        self._action = None
        self._baxterCommand = []

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, text):
        print ('output buffer received: ' + text)
        self._text = text

    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, a):
        self._action = a

    @property
    def baxterCommand(self):
        if self._baxterCommand:
            return self._baxterCommand.pop()
        else:
            return None

    @baxterCommand.setter
    def baxterCommand(self, val):
        assert val[0] in ['look', 'play', 'discard', 'point']
        self._baxterCommand = [val]

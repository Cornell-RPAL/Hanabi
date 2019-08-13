class OutputBuffer():
    def __init__(self):
        self._text = []
        self._action = None
        self._baxterCommand = []

    @property
    def text(self):
        if self._text:
            return self._text.pop()
        else:
            return None

    @text.setter
    def text(self, val):
        print (f'output buffer received: {val}')
        self._text = [val]

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

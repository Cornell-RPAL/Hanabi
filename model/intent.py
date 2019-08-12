from .action import PlaySuccess, PlayFail, Discard, Hint
from .consts import HANABOT

class Intent:
    """
    An interface that represents the intent of Hanabot.

    When completed, it should be transformed to an [Action] object.
    """
    def __init__ (self, feature = None, indices = []):
        """
        [feature] is [None] iff action is [Play] or [Discard].
        """
        self._indices = indices
        self._feature = feature
        self._baxterCommand = ''

    def complete(self, cardList = [], success = True):
        if isinstance(self, PlayIntent):
            if success:
                return PlaySuccess(HANABOT, cardList[0], indices = self._indices)
            else:
                return PlayFail(HANABOT, cardList[0], indices = self._indices)
        elif isinstance(self, DiscardIntent):
            return Discard(HANABOT, cardList[0], indices = self._indices)
        elif isinstance(self, HintIntent):
            return Hint(HANABOT, cardList, feature = self._feature, \
                indices = self._indices)
        else:
            print('intent completion failed')

    def __str__ (self):
        return ''

class PlayIntent(Intent):
    @property
    def baxterCommand(self):
        return ('look', self._indices)

    def __str__(self):
        return " play card at index " + str(self._indices)+"\n"

class DiscardIntent(Intent):
    @property
    def baxterCommand(self):
        return ('discard', self._indices)

    def __str__(self):
        return " discard card at index " + str(self._indices) +"\n"

class HintIntent(Intent):
    @property
    def baxterCommand(self):
        return ('point', self._indices)

    def __str__(self):
        idx = ""
        for num in self._indices:
            idx += " "+ str(num)
        return "Hint: your cards at indices "+ idx + " are "+ str(self._feature) +"\n"

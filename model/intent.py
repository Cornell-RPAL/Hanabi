from .action import PlaySuccess, PlayFail, Discard, Hint
from .consts import HANABOT
from log import log

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

    def __eq__ (self, other):
        if self._indices == other._indices and self._feature == other._feature \
            and self._baxterCommand == other._baxterCommand:
            return True

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
            log('intent completion failed')

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

    @property
    def feature(self):
        return self._feature


    def __str__(self):
        return "Hint: your cards at indices "+ str(self._indices) + " are "+ str(self._feature) +"\n"

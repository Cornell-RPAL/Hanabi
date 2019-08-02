from .consts import HANABOT

class Action:
    """
    An interface that represents the action of a player.
    """
    def __init__ (self, player, card = None, feature = None, indices=[]):
        """
        [feature] is [None] iff action is [Play] or [Discard].
        """
        self._player = player
        self._indices = indices
        self._feature = feature
        self._card = card
        self._baxterCommand = ''

    @property
    def card(self):
        return self._card

    @property
    def indices(self):
        return self._indices

class PlaySuccess(Action):
    pass


class PlayFail(Action):
    pass


class Discard(Action):
    pass


class Hint(Action):
    pass

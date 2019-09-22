from model.action import Action, PlaySuccess, PlayFail, Discard
from model.card import Card
from model.consts import PLAYER
from commandParser import CommandParser
from vision.frameStream import FrameStream
import asyncio
from log import log

TEXT_BUFFER_LENGTH = 10

class SensoryBuffer():
    def __init__(self, frame_num=30):
        # inputs and buffers

        # voice
        self._text = ''
        self._textHistory = []

        # cv
        self._initial_state = FrameStream().initial_state
        self._permanence = 0
        self._cvState = self._initial_state
        self._cvStateHistory = []

        #
        self._action = []

    @property
    def text(self):
        return self._text

    def setText(self, text):
        log("Set in a buffer: " + text)
        self._text = text
        if len(self._textHistory) >= TEXT_BUFFER_LENGTH:
            self._textHistory.pop(0)
        self._textHistory.append(text)

    @property
    def action(self):
        if self._action:
            return self._action.pop()
        else:
            return None

    @action.setter
    def action(self, value):
        if value is not None:
            if not isinstance(value, Action):
                raise Exception(f"{value} is not an Action object")
            self._action += [value]

    @property
    def cvState(self):
        return self._cvState

    def getGripper(self):
        g = self._cvState['gripper']
        if g:
            return g[0]
        else:
            return None


    @property
    def cvStateHistory(self):
        if self._cvStateHistory:
            return self._cvStateHistory.pop(0)
        else:
            return None

    @cvStateHistory.setter
    def cvStateHistory(self, new_state):
        self._cvStateHistory.append(new_state)

    def recognize_action(self, new_state):
        def stateChange(new_state): #maybe should change (does only hand matter??)
            visible = self.cvState
            visible['discard'] = [visible['discard'][0]]
            # log(visible, new_state)
            return visible != new_state

        def updateDiscard(new_state):
            self.cvState['discard'] = new_state['discard'] + self.cvState['discard']

        if not new_state:
            return None

        # should be still for 3 frames
        # check if exactly one card id in hand is different
        # log(self.cvState)
        if stateChange(new_state):
            log('detected state change')
            if new_state['gripper']:
                log(new_state['gripper'])
                self._cvState = new_state
            elif self._permanence > 5: #should set in const later
                self._permanence = 0
                if len(new_state['hand']) == 5:

                    new_hand = set(new_state['hand'])
                    old_hand = set(self.cvState['hand'])

                    if len(old_hand - new_hand) == 1:
                        action_card = (old_hand - new_hand).pop()
                        if action_card in new_state['board']:
                            log(self.cvState, new_state)
                            self._cvState = new_state
                            log('played', action_card)
                            log('new stable state:', self.cvState)
                            indices = [new_state['board'].index(action_card)]
                            return PlaySuccess(
                                PLAYER, action_card, indices=   indices)
                        if action_card in new_state['discard']: 
                            #could also just check top card
                            updateDiscard(new_state)

                            log('discarded', action_card)
                            log('new stable state:', self.cvState)
                            indices = [new_state['discard'].index(action_card)]
                            
                            return Discard(PLAYER, action_card, indices=indices)
            self._permanence += 1

    async def process(self):
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if self._text and self._text != oldText and self.cvStateHistory:
                log('input buffer processing...')
                self.action = CommandParser.parse(self._text)
                oldText = self._text
            self.action = self.recognize_action(self.cvStateHistory)



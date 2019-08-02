from model.action import Action
from model.card import Card
from commandParser import CommandParser
#from frameStream import FrameStream
import asyncio

TEXT_BUFFER_LENGTH = 10

class SensoryBuffer():
    def __init__(self, frame_num=30):
        # inputs and buffers

        # voice
        self._text = ''
        self._textHistory = []

        # cv
        self._permanence = 0
        self._prev_state = None
        self._cv_state_history = []

        # 
        self._action = [] 

    @property
    def text(self):
        return self._text

    def setText(self, text):
        print("Set in a buffer: " + text)
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
    def action(self, new_action):
        if new_action is not None:
            self._action += [new_action]


    @property
    def cv_state_history(self):
        if self._cv_state_history:
            return self._cv_state_history.pop()
        else:
            return None

    @cv_state_history.setter
    def _cv_state_history(self, new_state):
        self._state.append(new_state)



    async def process(self):
        oldText = ''
        prev_state = None
        while True:
            await asyncio.sleep(0.05)
            if self._text and self._text != oldText:
                print('input buffer processing...')
                self._action += [CommandParser.parse(self._text)]
                oldText = self._text
            self._action += recognize_action(self._prev_state, self.cv_state_history)

    def recognize_action(prev_state, new_state):

        def stateChange(prev_state, new_state): #maybe should change (does only hand matter??)
            visible = self.prev_state
            visible['discard'] = [visible['discard'][0]]
            return visible != new_state

        def updateDiscard(self, new_state):
            self.prev_state = {
            'discard': new_state['discard'] + self.prev_state['discard'],
            'hand': new_state['hand'],
            'board': new_state['board']
            }

        if new_state['gripper']:
            return 'attempt play', new_state['gripper'][0]

        # should be still for 3 frames
        # check if exactly one card id in hand is different

        if stateChange(prev_state, new_state):
            if self._permanence > 5: #should set in const later

                self._permanence = 0
                if len(new_state['hand']) == 5:

                    new_hand = set(new_state['hand'])
                    old_hand = set(self.prev_state['hand'])
        
                    if len(old_hand - new_hand) == 1:
                        action_card = (old_hand - new_hand).pop()
                        if action_card in new_state['board']:
                            self.prev_state = new_state
                            # print('played', action_card)
                            # print('new stable state:', prev_state)
                            return PlayCard(PLAYER, card_ix = new_state['board'].index(action_card))
                        if action_card in new_state['discard']: #could also just check top card
                            self._updateDiscard(new_state)
                            # print('discarded', action_card)
                            # print('new stable state:', self.prev_state)
                            return Discard(PLAYER, card_ix = new_state['discard'].index(action_card))
                        else:
                            pass
                            # print('a card just disappeared? very bad')
                else:
                    pass
                    # print('waiting to draw new card')
            else:
                pass
                # self._permanence += 1
                # print('waiting for still frame')
        else:
            pass
            # print('nothing is happening')
from model.action import Action
from model.card import Card
from commandParser import CommandParser
from frameStream import FrameStream
import asyncio

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
    def cvState(self):
        return self._cvState
    

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
            return visible != new_state

        def updateDiscard(self, new_state):
            self.cvState = {
            'discard': new_state['discard'] + self.cvState['discard'],
            'hand': new_state['hand'],
            'board': new_state['board']
            }

        print('state', new_state)
        if new_state is None:
            return []
        
        if new_state['gripper'] is not None:
            return 'attempt play', new_state['gripper']

        # should be still for 3 frames
        # check if exactly one card id in hand is different

        if stateChange(new_state):
            if self._permanence > 5: #should set in const later

                self._permanence = 0
                if len(new_state['hand']) == 5:

                    new_hand = set(new_state['hand'])
                    old_hand = set(self.cvState['hand'])
        
                    if len(old_hand - new_hand) == 1:
                        action_card = (old_hand - new_hand).pop()
                        if action_card in new_state['board']:
                            self.cvState = new_state
                            # print('played', action_card)
                            # print('new stable state:', self.cv2)
                            return PlayCard(PLAYER, card_ix = new_state['board'].index(action_card))
                        if action_card in new_state['discard']: #could also just check top card
                            self._updateDiscard(new_state)
                            # print('discarded', action_card)
                            # print('new stable state:', self.prev_state)
                            return Discard(PLAYER, card_ix = new_state['discard'].index(action_card))
        #                 else:
        #                     pass
        #                     print('a card just disappeared? very bad')
        #         else:
        #             pass
        #             print('waiting to draw new card')
        #     else:
        #         pass
        #         self._permanence += 1
        #         print('waiting for still frame')
        # else:
        #     pass
        #     print('nothing is happening')



    async def process(self):
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if self._text and self._text != oldText:
                print('input buffer processing...')
                self._action += [CommandParser.parse(self._text)]
                oldText = self._text
            self._action += self.recognize_action(self.cvStateHistory)

    

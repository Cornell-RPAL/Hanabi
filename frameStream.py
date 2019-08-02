
import numpy as np
import cv2
import asyncio  

from collections import Counter
from recognize_board import detectState, getTags
from model.action import Action, PlayCard, Discard
from model.card import Card
from model.consts import HANABOT, PLAYER


class FrameStream():

    def __init__(self):
        self.cap = cv2.VideoCapture(0)#USE 1 if personal computer, 0 if baxter workstation
        g_img = cv2.cvtColor(self.cap.read()[1], cv2.COLOR_BGR2GRAY)
        self.prev_state = detectState(getTags(g_img))
        print('Initial State:', self.prev_state)
        self._permanence = 0
        self.frame_num = 0

    def _updateDiscard(self, new_state):
        self.prev_state = {
        'discard': new_state['discard'] + self.prev_state['discard'],
        'hand': new_state['hand'],
        'board': new_state['board']
        }


    def stateChange(self, new_state): #maybe should change (does only hand matter??)
        visible = self.prev_state
        visible['discard'] = [visible['discard'][0]]
        return visible != new_state

    def card_played(self, new_state):
        return sum([i for i in range(5) if self.prev_state['hand'][i] == new_state['hand'][i]]) == 1



    def rec_action(self):
        ret, frame = self.cap.read()
        #may need to flip frame horizontally
        g_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        new_state = detectState(getTags(g_img))

        if new_state['gripper']:
            return 'attempt play', new_state['gripper'][0]

        # should be still for 3 frames
        # check if exactly one card id in hand is different

        if self.stateChange(new_state):
            if self._permanence > 5: #should set in const later

                self._permanence = 0

                if len(new_state['hand']) == 5:

                    new_hand = set(new_state['hand'])
                    old_hand = set(self.prev_state['hand'])
                    

                    
                    if len(old_hand - new_hand) == 1:
                        action_card = (old_hand - new_hand).pop()

                        if action_card in new_state['board']:
                            self.prev_state = new_state
                            print('played', action_card)
                            print('new stable state:', self.prev_state)
                            return PlayCard(PLAYER, card_ix = new_state['board'].index(action_card))

                        if action_card in new_state['discard']: #could also just check top card
                            self._updateDiscard(new_state)
                            print('discarded', action_card)
                            print('new stable state:', self.prev_state)
                            return Discard(PLAYER, card_ix = new_state['discard'].index(action_card))

                        else:
                            print('a card just disappeared? very bad')

                else:
                    print('waiting to draw new card')
            else:
                self._permanence += 1
                print('waiting for still frame')
        else:
            print('nothing is happening')
            return None


    async def frame_process(self, buffer, fps=10):
        print('Starting frame process...')
        while True:
            await asyncio.sleep(0.2)
            self.frame_num += 1
            action = self.rec_action()
            if action:
                print('Writing to buffer!')
                buffer.action = action


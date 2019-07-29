
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
        print('Initial State:')
        self.prev_state = detectState(getTags(g_img))
        self._permanence = 0
        self.frame_num = 0

    def _updateState(self, other):
        self.prev_state = {
        'discard_pile': other['discard'] + self.prev_state['discard'],
        'hand': other['hand'],
        'board': other['board']
        }

    def card_played(self, other):
        return sum([i for i in range(5) if self.prev_state['hand'][i] == other['hand'][i]]) == 1



    def rec_action(self):
        ret, frame = self.cap.read()
        #may need to flip frame horizontally
        g_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        new_state = detectState(getTags(g_img))

        # should be still for 3 frames
        # check if exactly one card id in hand is different

        if new_state != self.prev_state:
            if self._permanence > 5: #should set in const later
                self._permanence = 0
                new_set = set(new_state['hand'])
                old_set = set(self.prev_state['hand'])
                if len(new_set & old_set) == 4:
                    action_card = (new_set - old_set).pop()
                    if action_card in set(new_state['board']):
                        return PlayCard(PLAYER, action_card)
                    if action_card in set(new_state['discard']):
                        return DiscardCard(PLAYER, action_card)
                    else:
                        # should probably raise error here, for now will just ignore
                        pass
            else:
                self._permanence += 1
        else:
            # something is moving probably
            return None


    async def frame_process(self, buffer, fps=10):
        print('Starting frame process...')
        while True:
            await asyncio.sleep(0.05)
            print('frame ', self.frame_num)
            self.frame_num += 1
            action = self.rec_action()
            print(self.prev_state)
            if action:
                print('Writing to buffer!')
                buffer.setAction([action])


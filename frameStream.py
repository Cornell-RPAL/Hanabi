
import numpy as np
import cv2
from collections import Counter
from recognize_board import detectState, getTags
from model.action import Action, PlayCard, Discard
from model.card import Card
from model.consts import HANABOT, PLAYER


class FrameStream():

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.prev_state = detectState(getTags(self.cap.read()[1]))

    def _updateState(self, other):
        self.prev_state = {
        'discard': self.prev_state['discard'] + other['discard'],
        'hand': other['hand'],
        'board': other['board']
        }

    def rec_board_state(self):
        ret, frame = cap.read()
        new_state = detectState(getTags(frame))
        if new_state != prev_state:
            if Counter(new_state['hand']) != Counter(self.prev_state['hand']):
                used = [e not in prev_state['board'] for e in new_state['board']]
                if used:
                    used = used[0]
                #should probably throw an error here
                if Counter(new_state['board']) != Counter(self.prev_state['board']):
                    if used in new_state['board']:
                        _updateState(self.new_state)
                        return PlayCard(PLAYER, used)
                if Counter(new_state['discard']) != Counter(self.prev_state['discard']):
                    if used in new_state['discard']:
                        _updateState(self.new_state)
                        return Discard(PLAYER, used)
        return None

    async def frame_process(self, buffer, fps=10):
        while True:
            await asyncio.sleep(1/fps)
            buffer.setAction += [fs.checkFrame(frame)]


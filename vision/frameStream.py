
import numpy as np
import cv2
import asyncio  

from vision.recognize_board import detectState, getTags
from log import log


class FrameStream():

    def __init__(self):
        self.cap = cv2.VideoCapture(0)#USE 1 if personal computer, 0 if baxter workstation
        g_img = cv2.cvtColor(self.cap.read()[1], cv2.COLOR_BGR2GRAY)
        self.last_valid_state = detectState(getTags(g_img))
        self.frame_num = 0

    def initial_state(self):
        return self.last_valid_state

    async def frame_process(self, buffer, fps=10):
        log('Starting frame process...')
        while True:
            await asyncio.sleep(1)
            self.frame_num += 1
            ret, frame = self.cap.read()
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.initial_state = detectState(getTags(gray_img))
            buffer.cvStateHistory = self.last_valid_state

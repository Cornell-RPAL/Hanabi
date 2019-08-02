
import numpy as np
import cv2
import asyncio  

from recognize_board import detectState, getTags



class FrameStream():

    def __init__(self):
        self.cap = cv2.VideoCapture(0)#USE 1 if personal computer, 0 if baxter workstation
        # g_img = cv2.cvtColor(self.cap.read()[1], cv2.COLOR_BGR2GRAY)
        # self.prev_state = detectState(getTags(g_img))
        # print('Initial State:', self.prev_state)
        self.frame_num = 0

    async def frame_process(self, buffer, fps=10):
        print('Starting frame process...')
        while True:
            await asyncio.sleep(0.2)
            self.frame_num += 1
            ret, frame = self.cap.read()
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            buffer.cv_state_history.append(detectState(getTags(gray_img))


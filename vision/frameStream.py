
import numpy as np
import cv2
import asyncio  

from vision.recognize_board import detectState, getTags



class FrameStream():

    def __init__(self):
        self.cap = cv2.VideoCapture(0)#USE 1 if personal computer, 0 if baxter workstation
        g_img = cv2.cvtColor(self.cap.read()[1], cv2.COLOR_BGR2GRAY)
        self.initial_state = detectState(getTags(g_img))
        print('Initial State:', self.initial_state)
        self.frame_num = 0



    async def frame_process(self, buffer, fps=10):
        print('Starting frame process...')
        while True:
            await asyncio.sleep(0.2)
            self.frame_num += 1
            ret, frame = self.cap.read()
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            buffer.cvStateHistory = detectState(getTags(gray_img))
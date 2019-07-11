from model.action import Action
from commandParser import CommandParser
import asyncio

TEXT_BUFFER_LENGTH = 10

class SensoryBuffer():
    def __init__(self, frame_num=30):
        # inputs and buffers
        self._text = ''
        self._textHistory = []
        self._frames = []
        self._frame_num = frame_num

        # output
        self._action = None

    def setText(self, text):
        print("Set in a buffer: " + text)
        self._text = text
        if len(self._textHistory) >= TEXT_BUFFER_LENGTH:
            self._textHistory.pop(0)
        self._textHistory.append(text)

    @property
    def text(self):
        return self._text

    # def getFrame(self):
    #     return self._frames.pop()

    # def addFrame(self, frame):
    #     if len(self._frames >= frame_num:
    #         self._frames.pop(0)
    #     self._frames.append(frame)

    async def process(self):
        while True:
            await asyncio.sleep(0.05)
            self._action = CommandParser.parse(self._text)

    @property
    def action(self):
        a = self._action
        self._action = None
        return a


        


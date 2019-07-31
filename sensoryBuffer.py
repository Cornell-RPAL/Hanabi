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

        # output
        self._action = []


    def setText(self, text):
        print("Set in a buffer: " + text)
        self._text = text
        if len(self._textHistory) >= TEXT_BUFFER_LENGTH:
            self._textHistory.pop(0)
        self._textHistory.append(text)

    @property
    def text(self):
        return self._text

    async def process(self):
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if self._text and self._text != oldText:
                print ('input buffer processing...')
                self._action += [CommandParser.parse(self._text)]

    

    @property
    def action(self):
        if self._action:
            return self._action.pop()
        else:
            return None

    @action.setter
    def action(self, new_action):
        self._action += [new_action]

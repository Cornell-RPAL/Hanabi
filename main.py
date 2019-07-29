import asyncio
from multiprocessing import Process, Pipe

from sensoryBuffer import SensoryBuffer
from outputBuffer import OutputBuffer
from voice.voice_stream_to_text import main as v2tloop
from voice.gcloud_texttospeech import text_to_speech as t2s
from model.hanabot import Hanabot
from model.message import Message
from model.consts import HANABOT
from model.game import Game

try:
    from frameStream import FrameStream
except:
    pass

import argparse
parser = argparse.ArgumentParser()

class Main():
    def __init__(self, cv_off=False):
        self._sensoryBuffer = SensoryBuffer()
        self._oldLength = 1
        self._outputBuffer = OutputBuffer()
        self._game = Game()
        self._hanabot = Hanabot(self._game)
        if not cv_off:
            self._fs = FrameStream()
        
    async def _display(self):
        while True:
            await asyncio.sleep(0.5)
            #print (self._sensoryBuffer.text)
            if len(self._sensoryBuffer.text) != self._oldLength:
                print("Main.buffer change detected: ")
                print (self._sensoryBuffer.text)
                self._oldLength = len(self._sensoryBuffer.text)

    async def listen(self, end):
        while True:
            await asyncio.sleep(0.05)
            info = end.recv()
            if info:
                self._sensoryBuffer.setText(info)
    
    async def run(self, cv_off = False, voice_off = False):
        display = asyncio.create_task(self._display())

        if not voice_off:
            v2t_end, main_end = Pipe() # communication pipe for across processes
            listen = asyncio.create_task(self.listen(main_end))

            # multiprocessing is necessary for microphone stream
            v2t = Process(target = v2tloop, args = (v2t_end,))
            v2t.start()

        if not voice_off:
            t2v = asyncio.create_task(self.textToSpeech())

        if not cv_off:
            frame_processing = asyncio.create_task(self._fs.frame_process(self._sensoryBuffer, fps=10))

        input_processing = asyncio.create_task(self._sensoryBuffer.process())
        
        hanabot_processing = asyncio.create_task(
            self.runHanabot(self._sensoryBuffer, self._outputBuffer)
        )

        if not cv_off:
            await asyncio.gather(frame_processing)
        if not voice_off:
            await asyncio.gather(t2v, listen)

        await asyncio.gather(display, input_processing, hanabot_processing)

        if not voice_off:
            v2t.join()

    async def runHanabot(self, iBuffer, oBuffer):
        while True:
            await asyncio.sleep(0.05)
            observedAction = iBuffer.action
            if observedAction:
                # act in the game and inform hanabot
                observedAction.act(self._game, self._hanabot) 
                #self._hanabot.inform(iBuffer.action)

                action = self._hanabot.decideAction()
                oBuffer.action = action

                action.act(self._game, self._hanabot)

                m = Message()
                text = m.respond(action)

                oBuffer.text = text

    async def textToSpeech(self):
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if oldText != self._outputBuffer.text:
                t2s(self._outputBuffer.text)
                oldText = self._outputBuffer.text



parser.add_argument('-cv', action='store_true', help='no computer vision')
parser.add_argument('-v', action='store_true', help='no t2v or v2t')

if __name__ == '__main__':
    args = parser.parse_args()
    m = Main(cv_off=args.cv)
    asyncio.run(m.run(voice_off=args.v, cv_off=args.cv))
import asyncio
from multiprocessing import Process, Pipe

from sensoryBuffer import SensoryBuffer
from outputBuffer import OutputBuffer
from voice.voice_stream_to_text import main as v2tloop
from game.hanabi import Hanabot
from game.message import Message

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._oldLength = 1
        self._outputBuffer = OutputBuffer()
        self._game = Game()
        self._hanabot = Hanabot(self._game)
        
    async def _display(self):
        while True:
            await asyncio.sleep(0.5)
            #print (self._sensoryBuffer.text)
            if len(self._sensoryBuffer.text) != self._oldLength:
                print("Main.buffer change detected: ")
                print (self._sensoryBuffer.text)
                self._oldLength = len(self._ssensoryBuffer.text)

    async def listen(self, end):
        while True:
            await asyncio.sleep(0.05)
            info = end.recv()
            if info:
                self._sensoryBuffer.setText(info)
    
    async def run(self):
        display = asyncio.create_task(self._display())

        v2t_end, main_end = Pipe() # communication pipe for across processes
        listen = asyncio.create_task(self.listen(main_end))

        # As microphone needs to be on constantly, multiprocessing is necessary
        v2t = Process(target = v2tloop, args = (v2t_end,))
        v2t.start()

        input_processing = asyncio.create_task(self._sensoryBuffer.process())

        hanabot_processing = asyncio.create_task(
            self.runHanabot(self._sensoryBuffer, self._outputBuffer)
        )
    
        await asyncio.gather(display, listen, input_processing,\
            hanabot_processing)
        v2t.join()

    async def runHanabot(self, iBuffer, oBuffer):
        while True:
            if iBuffer.action:
                self._hanabot.inform(iBuffer.action)
            action = self._hanabot.decideAction()
            oBuffer.action = action

            m = Message()
            text = m.respond(action)

            oBuffer.text = text


m = Main()
asyncio.run(m.run())
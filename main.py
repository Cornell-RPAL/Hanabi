import asyncio
import psutil
from multiprocessing import Process, Pipe, Lock, Condition

from sensoryBuffer import SensoryBuffer
from vision.frameStream import FrameStream
from outputBuffer import OutputBuffer
from voice.voice_stream_to_text import main as v2tloop
from voice.text_to_speech import text_to_speech as t2s
from model.hanabot import Hanabot
from model.message import Message
from model.consts import HANABOT
from model.board import Board
from process_monitor import checkIfProcessRunning

class Main(object):
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._outputBuffer = OutputBuffer()
        self._hanabot = None
        self._childPid = -1
        self._fs = FrameStream()

    async def _listen(self, end):
        """
        Listens to the child processes for information.

        For right now, the only other process is speech to text, therefore
        it receives text from v2t and stores it in the sensoryBuffer.
        """
        while True:
            await asyncio.sleep(0.05)
            if end.poll():
                info = end.recv()
                if info:
                    self._sensoryBuffer.setText(info)

    async def _runHanabot(self):
        """
        Initializes the hanabot AI and keeps it running.
        """
        await asyncio.sleep(0.5)
        self._hanabot = Hanabot(Board(self._sensoryBuffer.cvState['hand']))
        await self._hanabot.react(self._sensoryBuffer, self._outputBuffer)

    async def textToSpeech(self):
        """
        Monitors and plays synthesized text in outputBuffer.
        """
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if oldText != self._outputBuffer.text:
                p = psutil.Process(self._childPid)
                p.suspend() # prevent computer from hearing itself
                print ('synthesizing text from output buffer')
                t2s(self._outputBuffer.text)
                p.resume()
                oldText = self._outputBuffer.text

    async def manageProcess(self, v2t, v2t_end):
        while True:
            await asyncio.sleep(0.02)
            print('managing processes')

            if checkIfProcessRunning('play'):
                print('play detected')
                v2t.terminate()
            elif not v2t.is_alive():
                v2t = Process(target = v2tloop, args = (v2t_end,))
                v2t.start()

    async def run(self):
        """
        Runs all the processes and tasks.
        """
        v2t_end, main_end = Pipe() # communication pipe for across processes
        listen = asyncio.create_task(self._listen(main_end))

        # multiprocessing is necessary for microphone stream
        v2t = Process(target = v2tloop, args = (v2t_end,))
        v2t.start()
        self._childPid = v2t.pid

        input_processing = asyncio.create_task(self._sensoryBuffer.process())

        hanabot_processing = asyncio.create_task(self._runHanabot())

        t2v = asyncio.create_task(self.textToSpeech())

        frame_processing = asyncio.create_task(self._fs.frame_process(self._sensoryBuffer, fps=10))

        process_managing = asyncio.create_task(
            self.manageProcess(v2t, v2t_end)
        )

        tasks = (listen, frame_processing, input_processing, \
                hanabot_processing, process_managing, t2v,)

        await asyncio.gather(*tasks)

        v2t.terminate()


if __name__ == '__main__':
    m = Main()
    asyncio.run(m.run())

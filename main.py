import asyncio
import psutil
import sys
from multiprocessing import Process, Pipe, Lock, Condition
from subprocess import Popen

from log import log
from sensoryBuffer import SensoryBuffer
from vision.frameStream import FrameStream
from vision.simulate_frameStream import SimulateFrameStream
from outputBuffer import OutputBuffer
from voice.voice_stream_to_text import main as v2tloop
from voice.sim_voice_stream_to_text import main as simv2tloop
from voice.text_to_speech import text_to_speech as t2s
from model.hanabot import Hanabot
from model.message import Message
from model.consts import HANABOT
from model.board import Board
from process_monitor import checkIfProcessRunning
from sys import argv


def loop(func):
    async def _func(*args, **kwargs):
        while True:
            await asyncio.sleep(0.05)
            func(*args, **kwargs)
    return _func

class Main(object):
    def __init__(self):
        args = sys.argv
        self._outputBuffer = OutputBuffer()
        self._hanabot = None
        self._childPid = -1
        self._fs = None
        for opt in args:
            if opt == "-sv":
                self._fs = SimulateFrameStream()
        if self._fs == None:
            self._fs = FrameStream()
        self._sensoryBuffer = SensoryBuffer(self._fs.initial_state())

    @loop
    def _listen(self, end):
        """
        Listens to the child processes for information.

        For right now, the only other process is speech to text, therefore
        it receives text from v2t and stores it in the sensoryBuffer.
        """
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

    @loop
    def textToSpeech(self):
        """
        Monitors and plays synthesized text in outputBuffer.
        """
        text = self._outputBuffer.text
        if text:
            p = psutil.Process(self._childPid)
            p.suspend() # prevent computer from hearing itself
            log('synthesizing text from output buffer')
            t2s(text)
            self._sensoryBuffer.justSpoke = True
            log('if you see this only after audio finishies, should be blocking')
            p.resume()

    async def manageProcess(self, v2t, v2t_end):
        detected = False
        while True:
            await asyncio.sleep(0.02)
            if not detected:
                log('managing processes')
            else:
                log("play detected!")

            #if checkIfProcessRunning('play'):
            #    detected = True
            #        log('play detected')
            #    v2t.terminate()
            #elif not v2t.is_alive():
            #    v2t = Process(target = v2tloop, args = (v2t_end,))
            #    v2t.start()

    @loop
    def _runBaxter(self):
        command = self._outputBuffer.baxterCommand
        if command:
            # baxter_env = ['cd ~/ros_ws && ./baxter.sh']
            fn = 'baxter/' + command[0] + '.py'
            typ = '--nargs-int-type' if command[0] == 'point' else ''
            if typ:
                args = ['python2.7'] + [fn] + [typ] + [str(i) for i in command[1]]
            else:
                args = ['python2.7'] + [fn] + [str(i) for i in command[1]]
            log(args)
            Popen(args)

    async def run(self, systemargument):
        """
        Runs all the processes and tasks.
        """
        v2t_send, main_rcv = Pipe() # communication pipe for across processes
        listen = asyncio.create_task(self._listen(main_rcv))

        # multiprocessing is necessary for microphone stream
        if ("-sv" not in systemargument):
            v2t = Process(target = v2tloop, args = (v2t_send,))
        else:
            v2t = Process(target = simv2tloop, args = (v2t_send,))

        v2t.start()
        self._childPid = v2t.pid

        input_processing = asyncio.create_task(self._sensoryBuffer.process())

        hanabot_processing = asyncio.create_task(self._runHanabot())

        t2v = asyncio.create_task(self.textToSpeech())

        frame_processing = asyncio.create_task(self._fs.frame_process(self._sensoryBuffer, fps=10))

        # process_managing = asyncio.create_task(self.manageProcess(v2t, v2t_end))

        baxter_running = asyncio.create_task(self._runBaxter())

        tasks = (listen, frame_processing, input_processing, \
                hanabot_processing, t2v, baxter_running)

        await asyncio.gather(*tasks)

        v2t.terminate()


if __name__ == '__main__':
    m = Main()
    asyncio.run(m.run(argv))

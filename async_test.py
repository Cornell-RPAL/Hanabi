from multiprocessing import Process, Queue, Pipe, Lock, Event
from voice.voice_stream_to_text import main as v2tloop
from voice.gcloud_texttospeech import text_to_speech as t2s
from sensoryBuffer import SensoryBuffer
import asyncio
import psutil

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._childPid = -1
        # self._notPlaying = Event()
        # self._notPlaying.set()
        pass

    async def display(self):
        while True:
            await asyncio.sleep(0.05)
            print('refresh')

    async def listen_queue(self):
        while True:
            await asyncio.sleep(0.05)

    async def textToSpeech(self):
        oldText = ''
        while True:
            await asyncio.sleep(0.05)
            if oldText != self._sensoryBuffer.text:
                print ('synthesizing text from output buffer')
                # self._notPlaying.clear()
                p = psutil.Process(self._childPid)
                p.suspend()
                t2s(self._sensoryBuffer.text)
                p.resume()
                print ('Finished speaking??')
                # self._notPlaying.set()
                oldText = self._sensoryBuffer.text

    async def listen(self, end):
        while True:
            await asyncio.sleep(0.05)
            print('listening...')
            if end.poll():
                info = end.recv()
                if info:
                    self._sensoryBuffer.setText(info)

    async def run(self):
        #q = Queue()
        
        v2t_end, main_end = Pipe()
        listen = asyncio.create_task(self.listen(main_end))
        v2t = Process(target = v2tloop, args = (v2t_end,))
        
        v2t.start()
        self._childPid = v2t.pid

        t2v = asyncio.create_task(self.textToSpeech())


        display = asyncio.create_task(self.display())
        await asyncio.gather(display, listen, t2v)
        

m = Main()
asyncio.run(m.run())
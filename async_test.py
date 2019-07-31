from multiprocessing import Process, Queue, Pipe, Lock
from voice.voice_stream_to_text import main as v2tloop
from sensoryBuffer import SensoryBuffer
import asyncio

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._q = Queue()
        pass

    async def display(self):
        while True:
            await asyncio.sleep(0.05)
            print('refresh')

    async def listen_queue(self):
        while True:
            await asyncio.sleep(0.05)
            

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


        display = asyncio.create_task(self.display())
        await asyncio.gather(display, listen)
        

m = Main()
asyncio.run(m.run())
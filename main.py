import asyncio
import concurrent.futures
from multiprocessing import Process, Queue, Pipe

from sensoryBuffer import SensoryBuffer
from voice.voice_stream_to_text import main as v2tloop

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._oldLength = 1
        
    async def _display(self):
        while True:
            await asyncio.sleep(0.5)
            print (self._sensoryBuffer.text)
            if len(self._sensoryBuffer.text) != self._oldLength:
                print("hi")
                print (self._sensoryBuffer.text)
                self._oldLength = len(self._sensoryBuffer.text)

    async def listen(self, end):
        while True:
            await asyncio.sleep(0.05)
            info = end.recv()
            if info:
                self._sensoryBuffer.setText(info)
    
    async def run(self):
        #v2t = asyncio.create_task(v2tloop(self._sensoryBuffer))
        d = asyncio.create_task(self._display())
        
        

        #executor = concurrent.futures.ProcessPoolExecutor()
        #executor.submit(v2tloop, self._sensoryBuffer)
        v2t_end, main_end = Pipe()
        l = asyncio.create_task(self.listen(main_end))
        p = Process(target = v2tloop, args = (v2t_end,))
        p.start()
        #self._sensoryBuffer.setText(main_end.recv())

        await asyncio.gather(d, l)
        p.join()
        # with concurrent.futures.ProcessPoolExecutor() as pool:
        #     await loop.run_in_executor(pool, v2tloop(self._sensoryBuffer))
        # with concurrent.futures.ProcessPoolExecutor() as pool:
        #     await loop.run_in_executor(pool, self._display)

m = Main()
#m.run()
#loop = asyncio.get_event_loop()
#loop.run_until_complete(m.run())

asyncio.run(m.run())
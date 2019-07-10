import asyncio
import concurrent.futures

from sensoryBuffer import SensoryBuffer
from voice.voice_stream_to_text import main as v2tloop

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()
        self._oldLength = 1
        

    async def _display(self):
        while True:
            await asyncio.sleep(0.5)
            print ("test")
            # if len(self._sensoryBuffer.text) != self._oldLength:
            #     print("hi")
            #     print (self._sensoryBuffer.text)
            #     self._oldLength = len(self._sensoryBuffer.text)
    
    async def run(self):
        #v2t = asyncio.create_task(v2tloop(self._sensoryBuffer))
        d = asyncio.create_task(self._display())
        

        executor = concurrent.futures.ProcessPoolExecutor()
        executor.submit(v2tloop, self._sensoryBuffer)
        #executor.submit(self._display)

        await asyncio.gather(d)
        # with concurrent.futures.ProcessPoolExecutor() as pool:
        #     await loop.run_in_executor(pool, v2tloop(self._sensoryBuffer))
        # with concurrent.futures.ProcessPoolExecutor() as pool:
        #     await loop.run_in_executor(pool, self._display)

m = Main()
#m.run()
#loop = asyncio.get_event_loop()
#loop.run_until_complete(m.run())

asyncio.run(m.run())
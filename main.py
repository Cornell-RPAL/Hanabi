import asyncio

from sensoryBuffer import SensoryBuffer
from voice.voice_stream_to_text import main as v2tloop

class Main():
    def __init__(self):
        self._sensoryBuffer = SensoryBuffer()

    async def _display(self):
        #while True:
        await asyncio.sleep(0.5)
        print ("test")
        print (self._sensoryBuffer.text)
    
    async def run(self):
        #v2t = asyncio.create_task(v2tloop(self._sensoryBuffer))
        d = asyncio.create_task(self._display())
        await asyncio.gather(d)

m = Main()
asyncio.run(m.run())
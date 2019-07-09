import asyncio

def timer(secs=1):
    def _timer(f):
        async def wrapper(*args, **kwargs):
            while 1:
                await asyncio.sleep(secs)
                await f()
        return wrapper
    return _timer

class DS():
    def __init__(self):
        self.prev_len = 0
        self.l = []

    async def f1(self):
        await asyncio.sleep(1)
        yield 42
        await asyncio.sleep(1)
        yield 43
        await asyncio.sleep(1)
        yield 44

    async def f2(self):
        await asyncio.sleep(2)
        yield 24
        await asyncio.sleep(2)
        yield 25

    async def t1(self):
        async for n in self.f1():
            self.l.append(n)

    async def t2(self):
        async for n in self.f2():
            self.l.append(n)
    
    async def display(self):
        while True:
            await asyncio.sleep(0.5)
            if len(self.l) > self.prev_len:
                print (self.l)
                self.prev_len = len(self.l)

    async def main(self):
        task1 = asyncio.create_task(self.t1())
        task2 = asyncio.create_task(self.t2())

        task3 = asyncio.create_task(self.display())
        
        await asyncio.gather(task1, task2, task3)
        #self.display()



d = DS()
asyncio.run(d.main())
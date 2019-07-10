#!/usr/bin/env python3

import asyncio

import speech_recognition as sr

async def listen_async(self, source):
    import threading
    result_future = asyncio.Future()
    def threaded_listen():
        with source as s:
            try:
                audio = self.listen(s)
                loop.call_soon_threadsafe(result_future.set_result, audio)
            except Exception as e:
                loop.call_soon_threadsafe(result_future.set_exception, e)
    listener_thread = threading.Thread(target=threaded_listen)
    listener_thread.daemon = True
    listener_thread.start()
    return await result_future

async def run(buffer):
    text = ''
    while text != 'quit':
        r = sr.Recognizer()
        m = sr.Microphone()
        audio = await listen_async(r, m)
        try:
            text = r.recognize_google(audio)
        except sr.UnknownValueError:
            continue
        buffer.setText(text)


#loop = asyncio.get_event_loop()
#loop.run_until_complete(run())
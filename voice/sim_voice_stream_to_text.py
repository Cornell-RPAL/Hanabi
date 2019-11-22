from log import log
import asyncio

def main(sender):
    filename = "voice/simulatedVoice.txt"
    last_valid_text = ""
    while (True):
        voice_msg = open(filename, "r").readlines()[0]
        if voice_msg != last_valid_text:
            log(f"generated {voice_msg}")
            sender.send(voice_msg)
            last_valid_text = voice_msg

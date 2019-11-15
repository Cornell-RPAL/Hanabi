from log import log
import time

def main(sender):
    while (True):
        try:
            text = input("input: ")
            print(f"generated {text}")
            sender.send(text)
        except EOFError:
            print("generated EOFError")
            time.sleep(5)
            pass 

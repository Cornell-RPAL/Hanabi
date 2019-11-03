
def main(sender):
    while (True):
        try:
            text = input()
            log(f"generated {text}")
            sender.send(text)
        except EOFError:
            pass 

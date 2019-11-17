
def main(sender):
    while (True):
        text = input()
        log(f"generated {text}")
        sender.send(text)

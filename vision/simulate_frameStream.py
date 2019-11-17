import asyncio
from model.card import Card
from log import log

class SimulateFrameStream(): 
    def __init__(self):
        self.filename = "vision/simulatedBoard.txt"
        f = open(self.filename, "r")
        if f.mode != "r":
            raise RuntimeError("simulateBoard.txt must be given read permissions")
        game_text = f.readlines()
        discard = [self.to_card(game_text[0])]
        hand = [self.to_card(item) for item in game_text[1].split(", ")]
        board = [self.to_card(item) for item in game_text[2].split(", ")] if len(game_text) >= 3 else []
        gripper = [self.to_card(game_text[3])] if len(game_text) >= 4 else []
        self.last_valid_board = {"discard": discard, "hand": hand, "board": board, "gripper": gripper}
    
    async def frame_process(self, buffer, fps):
        log("Starting simulation frame process..")
        while True:
            await asyncio.sleep(0.2)
            game_text = open(self.filename, "r").readlines()
            if len(game_text) < 3:
                return self.last_valid_board
            discard = [self.to_card(game_text[0])]
            hand = [self.to_card(item) for item in game_text[1].split(", ")]
            board = [self.to_card(item) for item in game_text[2].split(", ")] if len(game_text) >= 3 else []
            gripper = [self.to_card(game_text[3])] if len(game_text) >= 4 else []
            game_state_parsed = {"discard": discard, "hand": hand, "board": board, "gripper": gripper}
            if self.last_valid_board != game_state_parsed:
                self.last_valid_board = game_state_parsed
                log(self.last_valid_board)
            buffer.cvStateHistory = self.last_valid_board

    def to_card(self, card_name):
        colors = ["green", "blue", "red", "yellow", "white"]
        numbers = {1: 0, 2: 3, 3: 5, 4: 7, 5: 9} #Key: number, value: last digit of id
        args = card_name.split(" ")
        color = args[0].strip()
        number = int(args[1].strip())
        id = colors.index(color)*10 + numbers[number]
        return Card(color, number, id)

    def initial_state(self):
        return self.last_valid_board

from game.action import Hint

class CommandParser():
    def __init__(self):
        pass

    def parse(self, text):
        # HINT:
        # "Your card at index 3 is red" (currently support)
        # "Your first, second and last cards are white"

        word_list = ' '.split(text)
        if 'index' in word_list:
            i = word_list.index("index")
        elif 'indeces' in word_list:
            i = word_list.index("indeces")
        
        index_list = []
        while True:
            i += 1
            try:
                if word_list[i] == 'and':
                    continue
                n = int(word_list[i])
                index_list.append(n)
            except:
                break
        i += 1 # 'is', 'are'
        try:
            feature = int(word_list[i])
        except ValueError:
            feature = word_list[i]
        
        return 
            
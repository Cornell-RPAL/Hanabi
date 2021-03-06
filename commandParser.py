from model.action import Hint
from model.consts import HANABOT, PLAYER
from log import log

class CommandParser():
    def __init__(self):
        pass

    @staticmethod
    def parse(text):
        # HINT:
        # "Your card at index 3 is red" (currently support)
        # "Your first, second and last cards are white"

        word_list = text.split(' ')
        if 'index' in word_list:
            i = word_list.index("index")
        elif 'indices' in word_list:
            i = word_list.index("indices")
        else:
            return

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
        except IndexError:
            return
        # feature = word_list[-1]

        dn = {
            'one': 1,
            'two': 2,
            'three': 3,
            'four': 4,
            'five': 5
        }

        if feature in dn:
            feature = dn[feature]

        log('feature: ' + str(feature))
        log('index list: ' + str(index_list))

        return Hint(PLAYER, feature=feature, indices=index_list)

if __name__ == '__main__':
    cp = CommandParser()
    cp.parse('your card at index 3 4 and 5 are 2')

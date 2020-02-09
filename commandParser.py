from model.action import Hint
from model.consts import HANABOT, PLAYER
from log import log

class parsingError(Exception):
    """Exception raised for issues with parsing

    Attributes:
        input -- input expression in which the error occurred
        explanation -- explanation of the error
    """

    def __init__(self, input, explanation):
        self.input = input
        self.explanation = explanation

class CommandParser():
    def __init__(self):
        pass

    @staticmethod
    def parse(text, pointedIndices):
        # HINT:
        # "Your card at index 3 is red" (currently support)
        # "Your first, second and last cards are white"

        # Red, red, Red
        # this card is Red
        # these cards are Red
        # this is Red
        # these are red
        # red
        #


        feature_list = set('red', 'green', 'yellow', 'blue', 'white', 'one', 'two', 'three', 'four', 'five', '1', '2', '3', '4', '5')
        word_list = set(text.split())
        intersection = feature_list.intersection(word_list)
        if len(intersection) == 1:
            feature = intersection
        elif len(intersection) < 1:
            raise parsingError(text, "Feature was not found")
        else:
            raise parsingError(text, "Too many features mentioned")



##################### OLD CODE ##########################

        # word_list = text.split(' ')
        # if 'index' in word_list:
        #     i = word_list.index("index")
        # elif 'indices' in word_list:
        #     i = word_list.index("indices")
        # else:
        #     raise parsingError(text, "Index was not found.")
        #
        # index_list = []
        # while True:
        #     i += 1
        #     try:
        #         if word_list[i] == 'and':
        #             continue
        #         n = int(word_list[i])
        #         index_list.append(n)
        #     except:
        #         break
        # i += 1 # 'is', 'are'
        # try:
        #     feature = int(word_list[i])
        # except ValueError:
        #     feature = word_list[i]
        # except IndexError:
        #     return
        # # feature = word_list[-1]


        # dn = {
        #     'one': 1,
        #     'two': 2,
        #     'three': 3,
        #     'four': 4,
        #     'five': 5
        # }
        #
        # if feature in dn:
        #     feature = dn[feature]
        #
        # log('feature: ' + str(feature))
        # log('index list: ' + str(index_list))
        #
        # return Hint(PLAYER, feature=feature, indices=index_list)
##################### OLD CODE ##########################


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

        return Hint(PLAYER, feature=feature, indices=pointedIndices)

if __name__ == '__main__':
    cp = CommandParser()
    cp.parse('your card at index 3 4 and 5 are 2')

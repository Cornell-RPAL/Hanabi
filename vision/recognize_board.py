import apriltags3
import os
import cv2
from cv2 import imshow
from model.card import Card

visualization = False
current_loc = os.path.dirname(os.path.abspath(__file__))
print(current_loc)
file_name = 'turn0.jpg'
read_path = current_loc + "/vision/data/board_rec/before/" + file_name
save_path = current_loc + "/vision/data/board_rec/after/" + file_name


at_detector = apriltags3.Detector(families='tagStandard41h12',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=1)

time_num = 0
time_sum = 0


img = cv2.imread(read_path, cv2.IMREAD_GRAYSCALE)


def id_to_card(id_):

    assert id_ < 60
    assert id_ >= 0

    colors = ['green', 'blue', 'red', 'yellow', 'white']
    numbers = {
    0: 1,
    1: 1,
    2: 1,
    3: 2,
    4: 2,
    5: 3,
    6: 3,
    7: 4,
    8: 4,
    9: 5
    }

    color = colors[id_ // 10]
    number = numbers.get(id_ % 10)

    return Card(color, number, id_)



def detectState(tags, empty_draw_pile = False, discard_threshold=100, hand_threshold=50, nearness_threshold=1.5):

    def find_center(tag):
        return sum(tag.corners)/4

    def find_width(tag):
        return (tag.corners[1][0] - tag.corners[0][0]) #* (tag.corners[2][1] - tag.corners[0][1])


    width_sorted = sorted(tags, key=find_width)
    if empty_draw_pile and (find_center(width_sorted[-1])[1] > (find_center(width_sorted[-2])[1] * nearness_threshold)):
        gripper = width_sorted[-1]
    else:
        gripper = []    

    # print('BIGGEST', id_to_card(width_sorted[-1].tag_id), find_width(width_sorted[-1]))
    # print('2nd BIG', id_to_card(width_sorted[-2].tag_id), find_width(width_sorted[-2]))

    y_sorted = sorted(tags, key=lambda tag: find_center(tag)[1])
    x_sorted = sorted(tags, key=lambda tag: find_center(tag)[0])
    if empty_draw_pile and (find_center(y_sorted[4])[1] > (find_center(y_sorted[3])[1] + hand_threshold)):
        hand = sorted(y_sorted[:4], key=lambda tag: find_center(tag)[0])
    else:
        hand = sorted(y_sorted[:5], key=lambda tag: find_center(tag)[0])

    # print(id_to_card(x_sorted[-1].tag_id), find_center(x_sorted[-1]))
    # print(id_to_card(x_sorted[-2].tag_id), find_center(x_sorted[-2]))

    if find_center(x_sorted[-1])[0] > (find_center(x_sorted[-2])[0] + discard_threshold):
        discard = [x_sorted[-1]]
        assert discard not in hand
    else:
        discard = []

    board = [tag for tag in x_sorted if ((tag not in hand) and (tag not in discard))]

    res = {"discard": discard, "hand": hand, "board": board, "gripper": gripper}

    for key in res:
        res[key] = [id_to_card(tag.tag_id) for tag in res[key]]
    print(res)
    return res



def getTags(img, flip=False, verbose=False, save=False):
    
    if flip:
        res = cv2.flip(img, 1)
    else:
        res = img

    tags = at_detector.detect(res)
    color_img = cv2.cvtColor(res, cv2.COLOR_GRAY2RGB)

    if verbose:
        tag_ids = [tag.tag_id for tag in tags]
        print(len(tags), " tags found.")
        print(len(tags), " cards found: ", [id_to_card(id_) for id_ in tag_ids])

    if save:
        cv2.imwrite(save_path, color_img)

    return tags
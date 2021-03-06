import apriltags3
import os
import cv2
import numpy as np
import math 
import numpy as np
from cv2 import imshow
from model.card import Card
from log import log

visualization = False
current_loc = os.path.dirname(os.path.abspath(__file__))
log(current_loc)
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



def detectState(tags, empty_draw_pile = False, discard_threshold=50, hand_threshold=50, nearness_threshold=1.5):

    def find_center(tag):
        return sum(tag.corners)/4

    def find_area(tag):
        return abs((tag.corners[0][0] - tag.corners[1][0]) * (tag.corners[1][1] - tag.corners[2][1])) #* (tag.corners[2][1] - tag.corners[0][1])


    area_sorted = sorted(tags, key=find_area)
    


    # log('cards:, ', [id_to_card(tag.tag_id) for tag in area_sorted])

    # log(find_area(area_sorted[-2]))
    # log(find_area(area_sorted[-1]))

    gripper = []
    if len(area_sorted) > 2 and (find_area(area_sorted[-1]) > (find_area(area_sorted[-2]) * 4)):
        log('comp1 ', id_to_card(area_sorted[-1].tag_id), find_area(area_sorted[-1]))
        log('comp2 ', id_to_card(area_sorted[-2].tag_id), find_area(area_sorted[-2]))
        gripper = [area_sorted[-1]]
        print("Gripper detected: ", gripper)

    # if empty_draw_pile and (find_center(area_sorted[-1])[1] > (find_center(area_sorted[-2])[1] * nearness_threshold)):
    #     gripper = area_sorted[-1]
    # else:
    #     gripper = []    

    # log('BIGGEST', id_to_card(width_sorted[-1].tag_id), find_width(width_sorted[-1]))
    # log('2nd BIG', id_to_card(width_sorted[-2].tag_id), find_width(width_sorted[-2]))

    y_sorted = sorted(tags, key=lambda tag: find_center(tag)[1])
    x_sorted = sorted(tags, key=lambda tag: find_center(tag)[0])

    # log('y', [id_to_card(tag.tag_id) for tag in y_sorted])
    # log('x', [id_to_card(tag.tag_id) for tag in x_sorted])

    if empty_draw_pile and (find_center(y_sorted[4])[1] > (find_center(y_sorted[3])[1] + hand_threshold)):
        hand = sorted(y_sorted[:4], key=lambda tag: find_center(tag)[0])
    else:
        hand = sorted(y_sorted[:5], key=lambda tag: find_center(tag)[0])

    # log(id_to_card(x_sorted[-1].tag_id), find_center(x_sorted[-1]))
    # log(id_to_card(x_sorted[-2].tag_id), find_center(x_sorted[-2]))

    if find_center(x_sorted[-1])[0] > (find_center(x_sorted[-2])[0] + discard_threshold):
        discard = [x_sorted[-1]]
        assert discard not in hand
    else:
        discard = []

    board = [tag for tag in x_sorted if ((tag not in hand) and (tag not in discard))]

    res = {"discard": discard, "hand": hand, "board": board, "gripper": gripper}

    for key in res:
        if res[key]:
            res[key] = [id_to_card(tag.tag_id) for tag in res[key]]
    log([key + ": " + ", ".join([str(card) for card in res[key]]) for key in res]) 
    return res



def getTags(img, flip=False, verbose=False, save=False):
    
    if flip:
        res = cv2.flip(img, 1)
    else:
        res = img

    #get camera params ---------------------------------------------------------
    x_rad = 640/2 #camera radius y direction
    y_rad = 480/2 #camera radius x direction
    fov = 60 #field of view (degrees)
    x_foc = x_rad / math.tan(fov/2) #calculate x focal length
    y_foc = y_rad / math.tan(fov/2) #calculate y focal length 
    cameraMatrix = np.array([x_foc, 0, x_rad, 0, y_foc, y_rad, 0, 0, 1]).reshape(3,3)
    cam_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2])

    tags = at_detector.detect(res, estimate_tag_pose=True, camera_params=cam_params, tag_size=0.05)
    print(tags)
    for tag in tags:
        print("Tag id:" + str(tag.tag_id))
        print("Tag pose_t:" + str(tag.pose_t)) 

    color_img = cv2.cvtColor(res, cv2.COLOR_GRAY2RGB)

    if verbose:
        tag_ids = [tag.tag_id for tag in tags]
        log(len(tags), " tags found.")
        log(len(tags), " cards found: ", [id_to_card(id_) for id_ in tag_ids])

    if save:
        cv2.imwrite(save_path, color_img)

    return tags

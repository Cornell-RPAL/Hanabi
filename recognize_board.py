import apriltags3
import os
import cv2
from cv2 import imshow

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
    0: '1',
    1: '1',
    2: '1',
    3: '2',
    4: '2',
    5: '3',
    6: '3',
    7: '4',
    8: '4',
    9: '5'
    }
    color = colors[id_ // 10]
    number = numbers.get(id_ % 10)

    return color, number

def flatness(points):
    assert len(points) == 4
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    xs.sort()
    ys.sort()
    return (sum(ys[2:])-sum(ys[:2]))/(sum(xs[2:])-sum(xs[:2]))

def center(tag):
    return sum(tag.corners)/4

def detectState(tags):
    hand = []
    board = []
    discard = None
    for t in tags:
        if center(t)[0] < 300:
            discard = [id_to_card(t.tag_id)]
        if center(t)[1] < 750:
            hand.append(id_to_card(t.tag_id))
        else:
            board.append(id_to_card(t.tag_id))
    return {"discard": discard, "hand": hand, "board": board}


def getTags(img, verbose=True, save=True, visualize=True):
    # res = cv2.flip(img, 1)
    res = img
    tags = at_detector.detect(res)
    color_img = cv2.cvtColor(res, cv2.COLOR_GRAY2RGB)

    # for tag in tags:
    #     for idx in range(len(tag.corners)):
    #         cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

    #     cv2.putText(color_img, str(tag.tag_id),
    #                 org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
    #                 fontFace=cv2.FONT_HERSHEY_SIMPLEX,
    #                 fontScale=0.8,
    #                 color=(0, 0, 255))

    if verbose:
        tag_ids = [tag.tag_id for tag in tags]
        print(len(tags), " tags found.")
        print(len(tags), " cards found: ", [id_to_card(id_) for id_ in tag_ids])

    if save:
        cv2.imwrite(save_path, color_img)

    # if visualize:
    #     cv2.imshow('Detected tags', color_img)
    #     k = cv2.waitKey(0)
    #     if k == 27:         # wait for ESC key to exit
    #             cv2.destroyAllWindows()

    return tags


#print(detectState(getTags(img, visualize=False)[0]))


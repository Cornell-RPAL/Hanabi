import apriltags3
import os
import cv2
from cv2 import imshow

visualization = False
current_loc = os.path.dirname(os.path.abspath(__file__))
print(current_loc)
read_path = current_loc + "/vision/data/at_test.JPG"
save_path = current_loc + "/vision/data/post_test.JPG"


at_detector = apriltags3.Detector(families='tagStandard41h12',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

print("\n\nTESTING WITH MULTIPLE TAGS IMAGES")

time_num = 0
time_sum = 0


img = cv2.imread(read_path, cv2.IMREAD_GRAYSCALE)

tags = at_detector.detect(img)

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


tag_ids = [tag.tag_id for tag in tags]
print(len(tags), " tags found: ", tag_ids)
print(len(tags), " cards found: ", [id_to_card(id_) for id_ in tag_ids])


color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

for tag in tags:
    for idx in range(len(tag.corners)):
        cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

    cv2.putText(color_img, str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))


if visualization:
    cv2.imshow('Detected tags', color_img)
    cv2.imwrite(save_path, color_img)

    k = cv2.waitKey(0)
    if k == 27:         # wait for ESC key to exit
            cv2.destroyAllWindows()
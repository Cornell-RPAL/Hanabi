# segment hand
import cv2
import os
import face_recognition
import random
import numpy as np
from recognize import detectState, getTags

current_loc = os.path.dirname(os.path.abspath(__file__))
print(current_loc)
file_name = 'pointing0.jpg'
read_path = current_loc + "/vision/data/board_rec/before/" + file_name
save_path = current_loc + "/vision/data/board_rec/after/" + file_name


def l2_dis(p1, p2):
    return math.sqrt((np.sum(p1 - p2)**2))

def segment_hand(fp, num_samples=300):
    image = face_recognition.load_image_file(fp)
    top, right, bottom, left = face_recognition.face_locations(image)[0]
    print(top, right, bottom, left)
    samples = [(random.randint(left, right), random.randint(top, bottom)) for i in range(num_samples)]
    sum_color = np.zeros(3)
    for sx, sy in samples:
        sum_color += image[sy][sx]

    sum_color = np.flip(sum_color, axis=0)
    sum_color = sum_color[np.newaxis, np.newaxis, :]
    print(sum_color.shape)

    edged=cv2.Canny(image,20,200)
    cv2.imshow('canny edges',edged)
    cv2.waitKey(0)
segment_hand(read_path)
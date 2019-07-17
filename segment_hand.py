# segment hand
import cv2
import os
from openpose import pyopenpose as op
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

    edged=cv2.Canny(image,10,200)
    cv2.imshow('canny edges',edged)
    cv2.waitKey(0)

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

# Read image and face rectangle locations
imageToProcess = cv2.imread(args[0].image_path)
handRectangles = [
    # Left/Right hands person 0
    [
    op.Rectangle(320.035889, 377.675049, 69.300949, 69.300949),
    op.Rectangle(0., 0., 0., 0.),
    ],
    # Left/Right hands person 1
    [
    op.Rectangle(80.155792, 407.673492, 80.812706, 80.812706),
    op.Rectangle(46.449715, 404.559753, 98.898178, 98.898178),
    ],
    # Left/Right hands person 2
    [
    op.Rectangle(185.692673, 303.112244, 157.587555, 157.587555),
    op.Rectangle(88.984360, 268.866547, 117.818230, 117.818230),
    ]
]

# Create new datum
datum = op.Datum()
datum.cvInputData = imageToProcess
datum.handRectangles = handRectangles

# Process and display image
opWrapper.emplaceAndPop([datum])
print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
cv2.imshow("OpenPose 1.5.0 - Tutorial Python API", datum.cvOutputData)
cv2.waitKey(0)
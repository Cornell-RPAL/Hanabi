import cv2
import pyopenpose as op
import numpy as np
import os

params = dict()
params["model_folder"] = "../../../openpose/models/"
params["hand"] = True

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()
datum = op.Datum()

point_left = []
point_right = []

point_l_path = "gesture_data/image_train/point_l_train/"
point_r_path = "gesture_data/image_train/point_r_train/"

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

def append_keypoints(folder, pose):
    keypoints = []
    for img in os.listdir(folder):
        img_process = cv2.imread(os.path.join(folder, img))
        datum.cvInputData = img_process
        opWrapper.emplaceAndPop([datum])
        if pose == 'left':
	        point_left.append(datum.handKeypoints[0])
        elif pose == 'right':
	        point_right.append(datum.handKeypoints[1])

append_keypoints(point_l_path, 'left')
append_keypoints(point_r_path, 'right')

point_left = np.asarray(point_left)
point_right = np.asarray(point_right)
np.save('gesture_data/point_left.npy', point_left)
np.save('gesture_data/point_right.npy', point_right)

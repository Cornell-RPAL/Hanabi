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
no_pose = []

no_pose_path = "gesture_data/image_train/no_pose"
point_l_path = "gesture_data/image_train/point_l_train/"
point_r_path = "gesture_data/image_train/point_r_train/"

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

def append_keypoints(folder, pose = "None"):
    keypoints = []
    for img in os.listdir(folder):
        img_process = cv2.imread(os.path.join(folder, img))
        datum.cvInputData = img_process
        opWrapper.emplaceAndPop([datum])
        p = np.append(datum.poseKeypoints, datum.handKeypoints[0])
        p = np.append(p, datum.handKeypoints[1])
        p = p.reshape(67, 3)
        if pose == 'left':
            point_left.append(p)
        elif pose == 'right':
            point_right.append(p)
        else:
        	no_pose.append(p)

append_keypoints(point_l_path, 'left')
append_keypoints(point_r_path, 'right')
append_keypoints(no_pose_path)

point_left = np.asarray(point_left)
point_right = np.asarray(point_right)
no_pose = np.asarray(no_pose)
np.save('gesture_data/point_left.npy', point_left)
np.save('gesture_data/point_right.npy', point_right)
np.save('gesture_data/no_pose.npy', no_pose)

import cv2
import pyopenpose as op
import numpy as np
import os

params = dict()
params["model_folder"] = "../../../openpose/models/"
params["hand"] = True
params["hand_detector"] = 2
params["body"] = 0

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()
datum = op.Datum()

point_left = []
point_right = []

point_l_path = "gesture_data/image_data/pointing_left/"
point_r_path = "gesture_data/image_data/pointing_right/"


opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

def append_keypoints(folder, pose):
    keypoints = []
    for img in os.listdir(folder):
        img_process = cv2.imread(os.path.join(folder, img))
        datum.cvInputData = img_process
        opWrapper.emplaceAndPop([datum])
        print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
        if pose == 'left':
	        point_left.append(datum.handKeypoints[0])
        elif pose == 'right':
	        point_right.append(datum.handKeypoints[1])

append_keypoints(point_l_path, 'left')
append_keypoints(point_r_path, 'right')

    # handRectangles = [
    #     # Left/Right hands person 0
    #     [
    #     op.Rectangle(320.035889, 377.675049, 69.300949, 69.300949),
    #     op.Rectangle(0., 0., 0., 0.),
    #     ],
    #     # Left/Right hands person 1
    #     [
    #     op.Rectangle(80.155792, 407.673492, 80.812706, 80.812706),
    #     op.Rectangle(46.449715, 404.559753, 98.898178, 98.898178),
    #     ],
    #     # Left/Right hands person 2
    #     [
    #     op.Rectangle(185.692673, 303.112244, 157.587555, 157.587555),
    #     op.Rectangle(88.984360, 268.866547, 117.818230, 117.818230),
    #     ]
    # ]

   # datum.handRectangles = handRectangles

    # Process and display image
#opWrapper.emplaceAndPop([datum])
#print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
#print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
#point_left.append(datum.handKeypoints[0])

#point_right_images = cv2.imread(point_l_path)
#datum.cvInputData = point_right_images
#opWrapper.emplaceAndPop([datum])
#print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
#print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
#point_left.append(datum.handKeypoints[0])

#cv2.waitKey(0)

point_left = np.asarray(point_left)
point_right = np.asarray(point_right)
np.save('point_left.npy', point_left)
np.save('point_right.npy', point_right)

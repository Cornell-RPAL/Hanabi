import keras
import pyopenpose as op
import numpy as np
import cv2
import time

from log import log
from keras.models import load_model

model = load_model('gesture_data/pointing.h5')

params = dict()
params["model_folder"] = "../../../openpose/models/"
params["hand"] = True

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()
datum = op.Datum()

cap = cv2.VideoCapture(0)

def indexFromKeypoint(x, y):
	if (y<250 or y>450):
		return -1
	elif x >= 190 and x <= 235:
		return 0
	elif x >= 241 and x <= 283:
		return 1
	elif x >= 288 and x <= 326:
		return 2
	elif x >= 328 and x <= 369:
		return 3
	elif x >= 378 and x <= 430:
		return 4
	else: return -1

timing = False
curpose = -1
start_time = time.time()

indices = []

def getPointingIndices():
	temp = indices
	indices = []
	timing = False
	curpose = -1
	return temp

while True:
	ret, frame = cap.read()

	datum.cvInputData = frame
	opWrapper.emplaceAndPop([datum])

	image = datum.cvOutputData

	cv2.imshow("OpenPose", image)

	if datum.poseKeypoints.any():
		try:
			p = np.append(datum.poseKeypoints, datum.handKeypoints[0])
			p = np.append(p, datum.handKeypoints[1])
			p = p.reshape(1, 67, 3)
			p = keras.utils.normalize(p, axis = 1)

			opt = model.predict_classes(p, batch_size = 67)
			for j in opt: 
				if j == 0:
<<<<<<< HEAD
					print("NO POSE")
					curpose = -1
					timing = False
				elif j == 1:
					print("point_left")
					x, y, _ = datum.handKeypoints[0][0][8]
					index = indexFromKeypoint(x,y)
					timing = index != -1
					if (timing and curpose == index):
						if time.time() - start_time > 0.3 and index not in indices:
							timing = True
							indices.append(index)
					elif (timing):
						curpose = index
						start_time = time.time()
					else: start_time = time.time()
				elif j == 2:
					print("point_right")
					x, y, _ = datum.handKeypoints[1][0][8]
					index = indexFromKeypoint(x,y)
					timing = index != -1
					if (timing and curpose == index):
						if time.time() - start_time > 0.3 and index not in indices:
							timing = True
							indices.append(index)
					elif (timing):
						curpose = index
						start_time = time.time()
					else: start_time = time.time()
			print (indices)
=======
					log("NO POSE")
				elif j == 1:
					log("point_left")
					keypoint = datum.handKeypoints[0][0][8]
					log(keypoint)
				elif j == 2:
					log("point_right")
					keypoint = datum.handKeypoints[1][0][8]
					log(keypoint)
>>>>>>> dc9fde9bf219bb818a6e93d888bcf35d2417527b
		except:
			continue
	key = cv2.waitKey(1)
	if key == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

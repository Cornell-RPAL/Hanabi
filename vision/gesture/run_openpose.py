import keras
import pyopenpose as op
import numpy as np
import cv2

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
					print("NO POSE")
				elif j == 1:
					print("point_left")
					keypoint = datum.handKeypoints[0][0][8]
					print(keypoint)
				elif j == 2:
					print("point_right")
					keypoint = datum.handKeypoints[1][0][8]
					print(keypoint)
		except:
			continue
	key = cv2.waitKey(1)
	if key == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

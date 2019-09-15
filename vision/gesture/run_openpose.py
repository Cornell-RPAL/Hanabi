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


indices = []
frameCount = 0

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
					x, y, _ = datum.handKeypoints[0][0][8]
					index = indexFromKeypoint(x,y)
					print(index)
					if index == -1:
						frameCount = 0
					else:
						frameCount+=1
					if frameCount >= 4 and index != -1 or index in indices:
						frameCount = 0
						if index not in indices and index != -1:
							indices.append(index)
				elif j == 2:
					frameCount+=1
					print("point_right")
					x, y, _ = datum.handKeypoints[1][0][8]
					index = indexFromKeypoint(x,y)
					print(index)
					if index == -1:
						frameCount = 0
					else:
						frameCount+=1
					if frameCount >= 4 and index != -1 or index in indices:
						frameCount = 0
						if index not in indices and index != -1:
							indices.append(index)
			print (indices)
		except:
			continue
	key = cv2.waitKey(1)
	if key == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

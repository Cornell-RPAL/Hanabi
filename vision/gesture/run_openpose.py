import keras
import pyopenpose as op
import numpy as np
import cv2
import time
import asyncio
from log import log
from keras.models import load_model

class GestureRec(object):

	def __init__(self):
		self.model = load_model('/home/cornell.edu/nst45/hanabi_project/Hanabi/vision/gesture/gesture_data/pointing.h5')

		self.params = dict()
		self.params["model_folder"] = "/home/cornell.edu/nst45/hanabi_project/openpose/models/"
		self.params["hand"] = True

		# Starting OpenPose
		self.opWrapper = op.WrapperPython()
		self.opWrapper.configure(self.params)
		self.opWrapper.start()
		self.datum = op.Datum()

		self.cap = cv2.VideoCapture(0)

		self.timing = False
		self.curpose = -1
		self.start_time = time.time()

		self.indices = []

	def indexFromKeypoint(self, x, y):
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

	# def getPointingIndices():
	# 	temp = indices
	# 	indices = []
	# 	timing = False
	# 	curpose = -1
	# 	return temp

	async def main(self):
		while True:
			#asyncio.sleep(0.05)
			ret, frame = self.cap.read()


			self.datum.cvInputData = frame
			self.opWrapper.emplaceAndPop([self.datum])

			#image = datum.cvOutputData
			#cv2.imshow("OpenPose", image)

			if self.datum.poseKeypoints.any():
			
				try:
					p = np.append(self.datum.poseKeypoints, self.datum.handKeypoints[0])
					p = np.append(p, self.datum.handKeypoints[1])
					p = p.reshape(1, 67, 3)
					p = keras.utils.normalize(p, axis = 1)
					opt = self.model.predict_classes(p, batch_size = 67)

					for j in opt: 
						if j == 0:
							print("NO POSE")
							self.curpose = -1
							self.timing = False
						elif j == 1:
							print("point_left")
							x, y, _ = self.datum.handKeypoints[0][0][8]
							index = self.indexFromKeypoint(x,y)
							self.timing = index != -1
							if (self.timing and self.curpose == index):
								if time.time() - self.start_time > 0.3 and index not in self.indices:
									self.timing = True
									self.indices.append(index)
							elif (self.timing):
								self.curpose = index
								self.start_time = time.time()
							else: self.start_time = time.time()
						elif j == 2:
							print("point_right")
							x, y, _ = self.datum.handKeypoints[1][0][8]
							index = self.indexFromKeypoint(x,y)
							self.timing = index != -1
							if (self.timing and self.curpose == index):
								if time.time() - self.start_time > 0.3 and index not in self.indices:
									self.timing = True
									self.indices.append(index)
							elif (self.timing):
								self.curpose = index
								self.start_time = time.time()
							else: self.start_time = time.time()
					print (self.indices)
				except ValueError:
					continue
				
			key = cv2.waitKey(1)
			if key == ord('q'):
				break


	async def run(self):
		x = asyncio.create_task(self.main())
		await asyncio.gather(x)

if __name__ == '__main__':
	g = GestureRec()
	asyncio.run(g.run())

	g.cap.release()
	cv2.destroyAllWindows()

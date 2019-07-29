import numpy as np
import cv2

cap = cv2.VideoCapture(1)
while True:
	ret, frame = cap.read()
	grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	print(grey.shape)
	cv2.imshow('frame', cv2.resize(grey, dsize=(640, 480), interpolation=cv2.INTER_CUBIC))

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
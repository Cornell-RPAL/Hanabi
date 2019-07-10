import numpy as np
import cv2
import rospy
import baxter_interface

rospy.init_node('practice_node')
l_gripper = baxter_interface.Gripper('left')


cap = cv2.VideoCapture(0)
while True:
	ret, frame = cap.read()
	cv2.imshow('frame', frame)

	if np.sum(frame) < 10000000:
		l_gripper.close(block=True)
	else:
		l_gripper.open(block=True)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
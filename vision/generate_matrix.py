import math 

x_rad = 640/2 #camera radius y direction
y_rad = 480/2 #camera radius x direction
fov = 60 #field of view (degrees)
x_foc = x_rad / math.tan(fov/2) #calculate x focal length
y_foc = y_rad / math.tan(fov/2) #calculate y focal length 
cameraMatrix = np.array([x_foc, 0, x_rad, 0, y_foc, y_rad, 0, 0, 1])
camparams = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2])

tags = at_detector.detect(img, estimate_tag_pose=True, camera_params=cam_params, tag_size=0.05)

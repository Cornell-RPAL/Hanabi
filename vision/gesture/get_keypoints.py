import cv2
import pyopenpose as op
import numpy as np

params = dict()
params["model_folder"] = "../../models/"
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

parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="../../../examples/media/COCO_val2014_000000000241.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
args = parser.parse_known_args()

# Add others in path?
for i in range(0, len(args[1])):
    curr_item = args[1][i]
    if i != len(args[1])-1: next_item = args[1][i+1]
    else: next_item = "1"
    if "--" in curr_item and "--" in next_item:
        key = curr_item.replace('-','')
        if key not in params:  params[key] = "1"
    elif "--" in curr_item and "--" not in next_item:
        key = curr_item.replace('-','')
        if key not in params: params[key] = next_item

# Construct it from system arguments
# op.init_argv(args[1])
# oppython = op.OpenposePython()

try:
    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    # Read image and face rectangle locations
    imageToProcess = cv2.imread(args[0].image_path)
    handRectangles = [
        # Left/Right hands person 0
        [
        op.Rectangle(320.035889, 377.675049, 69.300949, 69.300949),
        op.Rectangle(0., 0., 0., 0.),
        ],
        # Left/Right hands person 1
        [
        op.Rectangle(80.155792, 407.673492, 80.812706, 80.812706),
        op.Rectangle(46.449715, 404.559753, 98.898178, 98.898178),
        ],
        # Left/Right hands person 2
        [
        op.Rectangle(185.692673, 303.112244, 157.587555, 157.587555),
        op.Rectangle(88.984360, 268.866547, 117.818230, 117.818230),
        ]
    ]

    # Create new datum
    datum = op.Datum()
    datum.cvInputData = imageToProcess
    datum.handRectangles = handRectangles

    # Process and display image
    opWrapper.emplaceAndPop([datum])
    print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
    print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
    cv2.imshow("OpenPose 1.5.0 - Tutorial Python API", datum.cvOutputData)
    cv2.waitKey(0)
except Exception as e:
    # print(e)
    sys.exit(-1)


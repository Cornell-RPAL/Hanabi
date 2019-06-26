import apriltags3

visualization = True
try:
    import cv2
except:
    raise Exception('You need cv2 in order to run! However, you can still use the library without it.')

try:
    from cv2 import imshow
except:
    print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
    print("VIsualization will be disabled.")
    visualization = False

at_detector = apriltags3.Detector(families='tagStandard41h12',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

print("\n\nTESTING WITH MULTIPLE TAGS IMAGES")

time_num = 0
time_sum = 0


img = cv2.imread('/Users/noahthompson/Desktop/at_test.JPG', cv2.IMREAD_GRAYSCALE)

tags = at_detector.detect(img)


tag_ids = [tag.tag_id for tag in tags]
print(len(tags), " tags found: ", tag_ids)


color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

for tag in tags:
    for idx in range(len(tag.corners)):
        cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

    cv2.putText(color_img, str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))

if visualization:
    cv2.imshow('Detected tags', color_img)
    cv2.imwrite("/Users/noahthompson/Desktop/post_test.JPG", color_img)

    k = cv2.waitKey(0)
    if k == 27:         # wait for ESC key to exit
            cv2.destroyAllWindows()
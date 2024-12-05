import numpy as np
import cv2
from coneDetection_modules import *

img = cv2.imread("av_depthset/av_track13.png") # Read image from local repository
img_depth = cv2.imread("av_depthset/av_track_depth13.png", cv2.IMREAD_GRAYSCALE)
#img_depth = cv2.cvtColor(img_depth, cv2.COLOR_BGR2GRAY)

start_time = time.perf_counter()

img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionA(img, img_depth) # , img_depth

end_time = time.perf_counter()

print(end_time - start_time)
print(val_error)

cv2.imshow("RAW", img)
cv2.imshow("Blue cones", img_hxs_blue)
cv2.imshow("Yellow cones", img_hxs_yellow)
cv2.imshow("Cones", img_cones)

cv2.waitKey(0) # Press any button to continue...
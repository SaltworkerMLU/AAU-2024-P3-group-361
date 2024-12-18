import numpy as np
import cv2
from ComputerVision.coneDetection_modules import *

#img = cv2.imread("av_depthset/av_track23.png") # Read image from local repository
#img_depth = cv2.imread("av_depthset/av_track_depth23.png", cv2.IMREAD_GRAYSCALE)
img = cv2.imread("figures/av_depthset_2/av_track6Cd.png")
img_depth = cv2.imread("figures/av_depthset_2/av_track_depth6Cd.png", cv2.IMREAD_GRAYSCALE)
#img_depth = cv2.cvtColor(img_depth, cv2.COLOR_BGR2GRAY)

start_time = time.perf_counter()

img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionC(img, img_depth) # , img_depth

end_time = time.perf_counter()

print(end_time - start_time)
print(val_error)

#cv2.imwrite("av_track_cones_blue23.png", img_hxs_blue)
#cv2.imwrite("av_track_cones_yellow23.png", img_hxs_yellow)
#cv2.imwrite("av_track_cones23.png", img_cones)

cv2.imshow("RAW", img)
cv2.imshow("Blue cones", img_hxs_blue)
cv2.imshow("Yellow cones", img_hxs_yellow)
cv2.imshow("Orange cones", img_hxs_orange)
cv2.imshow("Cones", img_cones)

cv2.waitKey(0) # Press any button to continue...

clock = time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + "_" + str(time.time_ns() // 1000000)[9:13]
folder = "images_test1/"
cv2.imwrite(folder + "RGB_" + clock + ".png", img)
cv2.imwrite(folder + "depth_" + clock + ".png", img_depth)
cv2.imwrite(folder + "orange_" + clock + ".png", img_hxs_orange)
cv2.imwrite(folder + "cones_" + clock + ".png", img_cones)
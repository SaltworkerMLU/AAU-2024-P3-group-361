import numpy as np
import cv2
import time

"""
The function containing all the point processing done to get the cones.
Includes RGB to HSV conversion and algorithm to get BLOBs
""" 
def cone_pointProcessing(img):
    """
    Point processing
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # Convert BGR image to HSV

    # Do thresholding using hsv-image
    hue_blue = cv2.inRange(img_hsv[:,:,0],90, 120,cv2.THRESH_BINARY) # H(x,y) > 80
    hue_yellow = cv2.inRange(img_hsv[:,:,0], 15, 45, cv2.THRESH_BINARY) # H(x,y) > 80
    hue_orange = cv2.inRange(img_hsv[:,:,0], 0, 15, cv2.THRESH_BINARY) # H(x,y) > 80
    saturation_thresh = cv2.inRange(img_hsv[:,:,1],200,255,cv2.THRESH_BINARY) # S(x,y) > 200

    img_cones_blue = cv2.bitwise_and(hue_blue, saturation_thresh)
    img_cones_yellow = cv2.bitwise_and(hue_yellow, saturation_thresh)
    img_cones_orange = cv2.bitwise_and(hue_orange, saturation_thresh)

    return img_cones_blue, img_cones_yellow, img_cones_orange

"""
Get largest BLOB using grassfire-algorithm
"""
def get_largest_BLOB(img_cone):
    # Using grassfire-algorithm, find all non-connected BLOBs
    contours, _ = cv2.findContours(img_cone,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE )

    # SANITY CHECK: Ensure at least one BLOB exists
    if len(contours) == 0:
        return img_cone
    
    # Get largest BLOB
    maxContour = 0
    for contour in contours:
        contourSize = cv2.contourArea(contour)
        if contourSize > maxContour:
            maxContour = contourSize
            maxContourData = contour
    
    # SANITY CHECK: When no BLOB is registered, contourSize = 0
    if contourSize == 0:
        return img_cone

    # Create an empty image to fill out largest BLOB
    img_cone_largest = np.zeros_like(img_cone)
    img_cone_largest = cv2.fillPoly(img_cone_largest,[maxContourData],255)

    return img_cone_largest

"""
Get center of mass of BLOB images
"""
def cone_COM(img_cone, colour):
    # Apply grassfire-algorithm to find each pixel in BLOBs
    contours, _ = cv2.findContours(
        img_cone, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # SANITY CHECK: Ensure at least one BLOB exists
    if len(contours) == 0:
        img_cone = cv2.cvtColor(img_cone, cv2.COLOR_GRAY2BGR)

        # Blue cones are presumed to be to the left (first column)
        if colour == (255, 0, 0):
            cx = -len(img_cone[0])//2
            
        # Yellow cones are presumed to be to the right (law column)
        else:
            cx = int(len(img_cone[0]) * 1.5)
        return img_cone, cx
    
    # Get center of mass of only x-axis (y-axis is irrelevant)
    cx = 0
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            #cy = int(M['m01']/M['m00'])

    # Convert binary image to RGB image (only for visual aesthetics)
    img_cone = cv2.cvtColor(img_cone, cv2.COLOR_GRAY2BGR)
    cv2.line(img_cone, (cx, 0), (cx, img_cone.shape[1]), colour, 8) # Draw line

    return img_cone, cx

"""
Get center of rectangle engulfing BLOB
"""
def cone_rectangle_center(img_cone, colour):
    x_start,y_start,width,height = cv2.boundingRect(img_cone)

    x_end = x_start + width
    y_end = y_start + height

    # Blue
    if x_end == 0 and y_end == 0 and colour == (255, 0, 0):
        x_start = int(len(img_cone[0]) * 1.5)
        x_end = int(len(img_cone[0]) * 1.5)
    
    # Yellow
    if x_end == 0 and y_end == 0 and colour == (0, 255, 255):
        x_start = -len(img_cone[0])//2
        x_end = -len(img_cone[0])//2


    # Set thickness of drawn lines
    thickness = 8

    # Convert binary image to RGB image (only for visual aesthetics)
    img_cone = cv2.cvtColor(img_cone, cv2.COLOR_GRAY2BGR)

    # Draw result on input image
    cv2.rectangle(img_cone, (x_start, y_start), (x_end, y_end), colour, thickness) # Draw rectangle
    cv2.line(img_cone, ((x_start+x_end)//2, 0), ((x_start+x_end)//2, img_cone.shape[1]), colour, thickness) # Draw line

    return img_cone, (x_start+x_end)//2

"""
Get outer bound of rectangle engulfing BLOB
"""
def cone_rectangle_outerBound(img_cone, colour):
    x_start,y_start,width,height = cv2.boundingRect(img_cone)

    x_end = x_start + width
    y_end = y_start + height

    # Blue
    if x_end == 0 and y_end == 0 and colour == (255, 0, 0):
        x_start = int(len(img_cone[0]) * 1.5)
        x_end = int(len(img_cone[0]) * 1.5)
    
    # Yellow
    if x_end == 0 and y_end == 0 and colour == (0, 255, 255):
        x_start = -len(img_cone[0])//2
        x_end = -len(img_cone[0])//2

    # Set thickness of drawn lines
    thickness = 8

    # Convert binary image to RGB image (only for visual aesthetics)
    img_cone = cv2.cvtColor(img_cone, cv2.COLOR_GRAY2BGR)

    # Draw result on input image
    cv2.rectangle(img_cone, (x_start, y_start), (x_end, y_end), colour, thickness) # Draw rectangle
    
    if colour == (0, 255, 255): # Yellow line
        x_bound = x_start
        cv2.line(img_cone, (x_start, 0), (x_start, img_cone.shape[1]), colour, thickness) # Draw line
    elif colour == (255, 0, 0): # Blue line
        x_bound = x_end
        cv2.line(img_cone, (x_end, 0), (x_end, img_cone.shape[1]), colour, thickness) # Draw line
    else:
        assert False, "\"colour\" must be either blue (255,0,0) or yellow (0, 255, 255)"

    return img_cone, x_bound

"""
Get error value which the vehicle must navigate using
"""
def get_cone_error(img_cone, x_blue, x_yellow):
    # Set target
    target = img_cone.shape[1]//2

    # Draw line in center of image depicting target
    cv2.line(img_cone, (target, 0), (target, img_cone.shape[1]), (0, 0, 255), 8) # Red line

    # Draw line in center of cones, using x_yellow and x_blue, 
    x_pos = (x_yellow + x_blue) // 2
    cv2.line(img_cone, (x_pos, 0), (x_pos, img_cone.shape[1]), (255, 0, 255), 8) # Cyan line

    # Get error
    val_error = target-x_pos

    return img_cone, val_error

"""
RGB Method A
"""
def RGB_coneDectectionA(img, img_depth=0):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue_largest = get_largest_BLOB(img_hxs_blue)
    img_hxs_yellow_largest = get_largest_BLOB(img_hxs_yellow)

    img_hxs_blue_largest, x_blue = cone_rectangle_outerBound(img_hxs_blue_largest, (255, 0, 0))
    img_hxs_yellow_largest, x_yellow = cone_rectangle_outerBound(img_hxs_yellow_largest, (0, 255, 255))

    img_cones = img_hxs_blue_largest + img_hxs_yellow_largest

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange

"""
RGB Method B
"""
def RGB_coneDectectionB(img, img_depth=0):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue_largest = get_largest_BLOB(img_hxs_blue)
    img_hxs_yellow_largest = get_largest_BLOB(img_hxs_yellow)

    img_hxs_blue_largest, x_blue = cone_rectangle_center(img_hxs_blue_largest, (255, 0, 0))
    img_hxs_yellow_largest, x_yellow = cone_rectangle_center(img_hxs_yellow_largest, (0, 255, 255))

    img_cones = img_hxs_blue_largest + img_hxs_yellow_largest

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange

"""
RGB Method C
"""
def RGB_coneDectectionC(img, img_depth=0):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue_largest = get_largest_BLOB(img_hxs_blue)
    img_hxs_yellow_largest = get_largest_BLOB(img_hxs_yellow)

    img_hxs_blue_largest, x_blue = cone_COM(img_hxs_blue_largest, (255, 0, 0))
    img_hxs_yellow_largest, x_yellow = cone_COM(img_hxs_yellow_largest, (0, 255, 255))

    img_cones = img_hxs_blue_largest + img_hxs_yellow_largest

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange

"""
Depth Method A
"""
def depth_coneDectectionA(img, img_depth):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    #img_depth_thresh = np.where(img_depth>100, 255, 0).astype(np.uint8)
    img_depth_thresh = cv2.inRange(img_depth, 0, 75, cv2.THRESH_BINARY)
    img_hxs_blue = cv2.bitwise_and(img_hxs_blue, img_depth_thresh)
    img_hxs_yellow = cv2.bitwise_and(img_hxs_yellow, img_depth_thresh)
    img_hxs_orange = cv2.bitwise_and(img_hxs_orange, img_depth_thresh)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue, x_blue = cone_rectangle_outerBound(img_hxs_blue, (255, 0, 0))
    img_hxs_yellow, x_yellow = cone_rectangle_outerBound(img_hxs_yellow, (0, 255, 255))

    img_cones = img_hxs_blue + img_hxs_yellow

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange

"""
Depth Method B
"""
def depth_coneDectectionB(img, img_depth):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    #img_depth_thresh = np.where(img_depth>100, 255, 0).astype(np.uint8)
    img_depth_thresh = cv2.inRange(img_depth, 0, 75, cv2.THRESH_BINARY)
    img_hxs_blue = cv2.bitwise_and(img_hxs_blue, img_depth_thresh)
    img_hxs_yellow = cv2.bitwise_and(img_hxs_yellow, img_depth_thresh)
    img_hxs_orange = cv2.bitwise_and(img_hxs_orange, img_depth_thresh)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue, x_blue = cone_rectangle_center(img_hxs_blue, (255, 0, 0))
    img_hxs_yellow, x_yellow = cone_rectangle_center(img_hxs_yellow, (0, 255, 255))

    img_cones = img_hxs_blue + img_hxs_yellow

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange

"""
Depth Method C
"""
def depth_coneDectectionC(img, img_depth):
    img_hxs_blue, img_hxs_yellow, img_hxs_orange = cone_pointProcessing(img)

    #img_depth_thresh = np.where(img_depth>100, 255, 0).astype(np.uint8)
    img_depth_thresh = cv2.inRange(img_depth, 0, 75, cv2.THRESH_BINARY)
    img_hxs_blue = cv2.bitwise_and(img_hxs_blue, img_depth_thresh)
    img_hxs_yellow = cv2.bitwise_and(img_hxs_yellow, img_depth_thresh)
    img_hxs_orange = cv2.bitwise_and(img_hxs_orange, img_depth_thresh)

    kernel = np.ones((15,15),np.uint8)
    img_hxs_blue = cv2.morphologyEx(img_hxs_blue, cv2.MORPH_OPEN, kernel)
    img_hxs_yellow = cv2.morphologyEx(img_hxs_yellow, cv2.MORPH_OPEN, kernel)
    img_hxs_orange = cv2.morphologyEx(img_hxs_orange, cv2.MORPH_OPEN, kernel)

    img_hxs_blue, x_blue = cone_COM(img_hxs_blue, (255, 0, 0))
    img_hxs_yellow, x_yellow = cone_COM(img_hxs_yellow, (0, 255, 255))

    img_cones = img_hxs_blue + img_hxs_yellow

    img_cones, val_error = get_cone_error(img_cones, x_blue, x_yellow)

    return img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange
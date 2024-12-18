"""
This program makes use of a variable power for the driving module
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from G10_P3_modules import *
import gpiozero as gpio

servo = gpio.AngularServo(4)
motor = gpio.Motor(12, 13)

# Create a pipeline
pipeline = rs.pipeline()

# Create config object
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 90)  # Depth stream
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60)  # Color stream

# Start the pipeline
pipeline.start(config)

# Align color and depth frames
align = rs.align(rs.stream.color)

try:
    lap = 0.5
    motor.forward(0.1)
    while True: # lap <= 2
        time_start = time.perf_counter()
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()

        # Get color and depth frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame:
            continue

        # Convert raw RGB-images to numpy arrays with 8-bit values
        img_RGB = np.asanyarray(color_frame.get_data())

        # Convert raw depth-images to numpy arrays with 8-bit values
        img_depth_raw = np.asanyarray(depth_frame.get_data())

        # Clip raw depth-images depth to 5000mm
        img_depth_raw = np.clip(img_depth_raw, 0, 5000)

        # Scale depth to range of 8-bit
        img_depth = (img_depth_raw / 5000 * 255).astype(np.uint8)

        # Slice the RGB-image and depth-image to keep only the region of interest
        img_RGB = img_RGB[img_RGB.shape[0]//2:,:]
        img_depth = img_depth[img_depth.shape[0]//2:,:]

        img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionC(img_RGB, img_depth)

        servo.angle = val_error * 60/(img_RGB.shape[1]/2)
        motor.forward(0.1 - abs(val_error / (img_RGB.shape[1]/2)*0.066))

        # Save images taken by camera
        #clock = time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + "_" + str(time.time_ns() // 1000000)[9:13]
        #folder = "figures/images_test1/"
        #cv2.imwrite(folder + "RGB_" + clock + ".png", img_RGB)
        #cv2.imwrite(folder + "depth_" + clock + ".png", img_depth)
        #cv2.imwrite(folder + "orange_" + clock + ".png", img_hxs_orange)
        #cv2.imwrite(folder + "cones_" + clock + ".png", img_cones)

        # Show images
        #cv2.imshow('RAW', img_RGB)
        #cv2.imshow('Depth', Depth_image_normalized)
        #cv2.imshow('Orange cone', img_hxs_orange)
        #cv2.imshow('Cones', img_cones)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        orange_density = np.sum(img_hxs_orange)/(img_hxs_orange.shape[0] * img_hxs_orange.shape[1])
        
        # If current lap is about to end
        if orange_density > 2 and (lap*2)%2 == 1:
            lap += 0.5
        
        # If av enters next lap
        elif orange_density < 1 and (lap*2)%2 == 0:
            lap += 0.5
        
        time_end = time.perf_counter()
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

    # Hit the brakes for 1 second
    time_brake = time.perf_counter()
    while time.perf_counter() - time_brake < 1:
        motor.forward(0)
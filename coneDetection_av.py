import pyrealsense2 as rs
import numpy as np
import cv2
import time
from coneDetection_modules import *
from gpiozero import gpio

servo = gpio.Servo(4)
motor = gpio.Motor(12, 13)

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
pipeline.start()
config = rs.config()
config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 90) # Max framerate for depth is 90fps
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30) # Max framerate for RGB is 30fps
align = rs.align(rs.stream.color)

try:
    lap = 0.5
    #motor.forward(0.1)
    while True: # lap <= 2
        time_start = time.perf_counter()
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()

        # Get color frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        Depth_frame = aligned_frames.get_depth_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        Depth_image = np.asanyarray(Depth_frame.get_data())
        img_RGB = np.zeros_like(color_image, dtype=np.uint8)
        img_RGB = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        #img_RGB = color_image
        img_RGB = img_RGB[img_RGB.shape[0]//2:,:]

        Depth_image_scaled = np.clip(Depth_image, 0, 5000)  # Clip to 1000mm (or any max depth you expect)
        Depth_image_normalized = np.array((Depth_image_scaled / 5000.0 * 255).astype(np.uint8))
        Depth_image_normalized = Depth_image_normalized[Depth_image_normalized.shape[0]//2:,:]

        img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = RGB_coneDectectionA(img_RGB, Depth_image_normalized)

        servo.angle(val_error * 60/(img_RGB.shape[1]/2))
        motor.forward(0.1 - abs(val_error / (img_RGB.shape[1]/2)*0.066))

        #print(0.1 - abs(val_error / (img_RGB.shape[1]/2)*0.066))

        # Show images
        #cv2.imshow('RAW', img_RGB)
        #cv2.imshow('Depth', Depth_image_normalized)
        cv2.imshow('Orange cone', img_hxs_orange)
        #cv2.imshow('Cones', img_cones)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        orange_density = np.sum(img_hxs_orange)/(img_hxs_orange.shape[0] * img_hxs_orange.shape[0])
        
        # If current lap is about to end
        if orange_density > 2 and (lap*2)%2 == 1:
            lap += 0.5
        
        # If av enters next lap
        elif orange_density < 1 and (lap*2)%2 == 0:
            lap += 0.5
        
        time_end = time.perf_counter()

        #print(time_end-time_start)
        #print(lap)
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

import pyrealsense2 as rs
import numpy as np
import cv2
import time
from coneDetection_modules import *

test_ID = 'S1' # c = RGB; d = depth

# Create a pipeline
pipeline = rs.pipeline()

# Create config object
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # Depth stream
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # Color stream

# Start the pipeline
pipeline.start(config)

# Align color and depth frames
align = rs.align(rs.stream.color)

try:
    i = 0
    framerate_array = np.array([])

    # Use the first 20 frames to "wake up" the camera
    while i < 20:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        i+=1

    i = 0
    while i < 600:
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
        
        #Depth_colormap = cv2.applyColorMap(Depth_image_normalized, cv2.COLORMAP_JET)

        # Use all 6 methods one at a time
        #time_start = time.perf_counter()
        if i < 100:
            test_ID = test_ID[0:2] + 'Ac'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = RGB_coneDectectionA(img_RGB, img_depth)
        elif i < 200:
            test_ID = test_ID[0:2] + 'Bc'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = RGB_coneDectectionB(img_RGB, img_depth)
        elif i < 300:
            test_ID = test_ID[0:2] + 'Cc'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = RGB_coneDectectionC(img_RGB, img_depth)
        elif i < 400:
            test_ID = test_ID[0:2] + 'Ad'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionA(img_RGB, img_depth)
        elif i < 500:
            test_ID = test_ID[0:2] + 'Bd'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionB(img_RGB, img_depth)
        elif i < 600:
            test_ID = test_ID[0:2] + 'Cd'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionC(img_RGB, img_depth)
        #time_end = time.perf_counter()

        # Show images
        cv2.imshow('RAW', img_RGB)
        cv2.imshow('Depth', img_depth)
        cv2.imshow('Orange cones', img_hxs_orange)
        cv2.imshow('Cones', img_cones)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time_end = time.perf_counter()

        # Take a screenshot at the 50th frame using said method
        if i % 100 == 99:
            # Put framerate spanning 50 frames on RGB-image
            img_RGB = cv2.putText(img_RGB, 
                                        str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 
                                        1, (0, 255, 0), 1, cv2.LINE_AA)
            Depth_image_normalized = cv2.putText(img_depth, 
                                                    str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                                    1, (0, 255, 0), 1, cv2.LINE_AA)
            img_cones = cv2.putText(img_cones, 
                                    str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 1, cv2.LINE_AA)
            img_hxs_orange = cv2.putText(img_hxs_orange, 
                                    str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 1, cv2.LINE_AA)
            # Take sceenshots
            cv2.imwrite('./figures/av_depthset_2/av_track' + test_ID + '.png', img_RGB)
            cv2.imwrite('./figures/av_depthset_2/av_track_depth' + test_ID + '.png', Depth_image_normalized)
            cv2.imwrite('./figures/av_depthset_2/av_track_cones' + test_ID + '.png', img_cones)
            cv2.imwrite('./figures/av_depthset_2/av_track_orange' + test_ID + '.png', img_hxs_orange)

        if i % 100 == 99:
            framerate_array = np.array([])
        i+=1

        framerate_array = np.append(framerate_array, time_end-time_start)
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

print(len(framerate_array))
#print(np.average(framerate_array))
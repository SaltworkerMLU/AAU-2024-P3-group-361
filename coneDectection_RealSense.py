import pyrealsense2 as rs
import numpy as np
import cv2
import time
from coneDetection_modules import *

test_ID = '0' # c = RGB; d = depth

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
pipeline.start()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90) # Max framerate for depth is 90fps
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Max framerate for RGB is 30fps
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

        Depth_image_scaled = np.clip(Depth_image, 0, 5000)  # Clip to 1000mm (or any max depth you expect)
        Depth_image_normalized = np.array((Depth_image_scaled / 5000.0 * 255).astype(np.uint8))
        
        #Depth_colormap = cv2.applyColorMap(Depth_image_normalized, cv2.COLORMAP_JET)

        # Use all 6 methods one at a time
        #time_start = time.perf_counter()
        if i < 100:
            test_ID = test_ID[0] + 'Ac'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = RGB_coneDectectionA(img_RGB, Depth_image_normalized)
        elif i < 200:
            test_ID = test_ID[0] + 'Bc'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = RGB_coneDectectionB(img_RGB, Depth_image_normalized)
        elif i < 300:
            test_ID = test_ID[0] + 'Cc'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = RGB_coneDectectionC(img_RGB, Depth_image_normalized)
        elif i < 400:
            test_ID = test_ID[0] + 'Ad'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = depth_coneDectectionA(img_RGB, Depth_image_normalized)
        elif i < 500:
            test_ID = test_ID[0] + 'Bd'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = depth_coneDectectionB(img_RGB, Depth_image_normalized)
        elif i < 600:
            test_ID = test_ID[0] + 'Cd'
            img_cones, val_error, img_hxs_blue, img_hxs_yellow = depth_coneDectectionC(img_RGB, Depth_image_normalized)
        #time_end = time.perf_counter()

        # Show images
        cv2.imshow('RAW', img_RGB)
        cv2.imshow('Depth', Depth_image_normalized)
        cv2.imshow('Cones', img_cones)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time_end = time.perf_counter()

        # Take a screenshot at the 50th frame using said method
        if i % 100 == 50:
            # Put framerate spanning 50 frames on RGB-image
            img_RGB = cv2.putText(img_RGB, 
                                        str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 
                                        1, (0, 255, 0), 1, cv2.LINE_AA)
            Depth_image_normalized = cv2.putText(Depth_image_normalized, 
                                                    str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                                    1, (0, 255, 0), 1, cv2.LINE_AA)
            img_cones = cv2.putText(img_cones, 
                                    str(int(1/np.average(framerate_array))) + ' fps', (0, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 1, cv2.LINE_AA)
            # Take sceenshots
            cv2.imwrite('./av_depthset_2/av_track' + test_ID + '.png', img_RGB)
            cv2.imwrite('./av_depthset_2/av_track_depth' + test_ID + '.png', Depth_image_normalized)
            cv2.imwrite('./av_depthset_2/av_track_cones' + test_ID + '.png', img_cones)

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
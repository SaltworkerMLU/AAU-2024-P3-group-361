import pyrealsense2 as rs
import numpy as np
import cv2
import time

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

start_time = time.time()
def millis():
    return int((time.time() - start_time))

previous = 0
fps = 0

try:
    while True:
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

        Depth_image_scaled = np.clip(Depth_image, 0, 2000)  # Clip to 1000mm (or any max depth you expect)
        Depth_image_normalized = (Depth_image_scaled / 2000.0 * 255).astype(np.uint8)

        #Depth_colormap = cv2.applyColorMap(Depth_image_normalized, cv2.COLORMAP_JET)

        HSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        height, width = color_image.shape[:2]
        H = np.zeros((height, width), dtype=np.uint8)
        S = np.zeros((height, width), dtype=np.uint8)
        HPS = np.zeros((height, width), dtype=np.uint8)

        #hsv_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)

       # Create mask for H and S channels
        H = np.where(HSV[:, :, 0] > 80, 255, 0).astype(np.uint8)  # Hue mask with binary values (0 or 255)
        S = np.where(HSV[:, :, 1] > 200, 255, 0).astype(np.uint8)  # Saturation mask with binary values (0 or 255)

        # Combine H and S to create HxS (logical AND of H and S)
        HxS = cv2.bitwise_and(H, S)  # Intersection of H and S

        # Combine HxS and S to create HPS
        # Use addition to incorporate both HxS and S into the final result
        HPS = cv2.addWeighted(HxS, 0.5, S, 0.5, 0)  # Weighted average of HxS and S
        HPS = np.clip(HPS, 0, 255).astype(np.uint8)  # Ensure values are in the range [0, 255]
        kernel = np.ones((15,15),np.uint8)
        Open_HPS = cv2.morphologyEx(HPS, cv2.MORPH_OPEN, kernel)

        

        # Display the color image
        #cv2.imshow('Color', color_image)
        #cv2.imshow('Depth', Depth_colormap)
        cv2.imshow('Depth2', Depth_image_normalized)
        #cv2.imshow('HxS', HxS)
        #cv2.imshow('Opened HPS', Open_HPS)
        #cv2.imshow('H', H)
        #cv2.imshow('S', S)
        current = millis()
        if current > previous:
            print(fps)
            fps = 0
            previous = current

        else:
            fps = fps+1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

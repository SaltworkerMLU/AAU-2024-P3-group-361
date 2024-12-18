import pyrealsense2 as rs
import numpy as np
import cv2

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
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()

        # Get color and depth frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame:
            continue

        # Convert raw RGB-images to numpy arrays with 8-bit values
        img_BGR = np.asanyarray(color_frame.get_data())
        img_RGB = np.zeros_like(img_BGR, dtype=np.uint8)
        img_RGB = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2RGB)

        # Convert raw depth-images to numpy arrays with 8-bit values
        img_depth_raw = np.asanyarray(depth_frame.get_data())

        # Clip raw depth-images depth to 5000mm
        img_depth_raw = np.clip(img_depth_raw, 0, 5000)

        # Scale depth to range of 8-bit
        img_depth = (img_depth_raw / 5000 * 255).astype(np.uint8)

        # Apply color map to grayscaled depth-image
        depth_colormap = cv2.applyColorMap(img_depth, cv2.COLORMAP_JET)

        # Display the color image
        cv2.imshow('Color', img_RGB)
        cv2.imshow('Depth (color map)', depth_colormap)
        cv2.imshow('Depth (grayscaled)', img_depth)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
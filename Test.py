import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
pipeline.start()

# Wait for a frame from the camera
frames = pipeline.wait_for_frames()

# Print the frames
print(frames)

# Stop the pipeline
pipeline.stop()

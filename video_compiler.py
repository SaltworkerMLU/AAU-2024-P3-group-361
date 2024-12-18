import cv2
import os
from moviepy.editor import VideoFileClip, concatenate_videoclips, clips_array

"""
This function converts a set of sequential images into a video.
Works assuming images in sequence are stored by timestamp.
"""
def image_to_video_av2(image_folder, video_name, fps, filter=''):
    # Store path of images in folder as array
    images = [img for img in os.listdir(image_folder) if img.startswith(filter) and img.endswith(".png")]

    # Get image shape using 1st frame
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    """
    Create video file in local repository. 
    Video is stored in compressed format -> cv2.VideoWriter_fourcc(*'DIVX')
    """
    video = cv2.VideoWriter(f'{video_name}.avi', cv2.VideoWriter_fourcc(*'DIVX'), fps, (width, height))

    # Write created video file frame by frame
    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

image_to_video_av2('figures/image_test9', 'image_test9_RGB', 30, 'RGB')
image_to_video_av2('figures/image_test9', 'image_test9_depth', 30, 'depth')
image_to_video_av2('figures/image_test9', 'image_test9_cones', 30, 'cones')
image_to_video_av2('figures/image_test9', 'image_test9_orange', 30, 'orange')

clip_1 = VideoFileClip("image_test9_RGB.avi")
clip_2 = VideoFileClip("image_test9_depth.avi")
clip_3 = VideoFileClip("image_test9_cones.avi")
clip_4 = VideoFileClip("image_test9_orange.avi")

# Clip each video together
final_clip = clips_array([[clip_1, clip_2], [clip_3, clip_4]])

# Write merged video file
final_clip.write_videofile("image_test9_merged.avi", fps=30, codec='libx264')

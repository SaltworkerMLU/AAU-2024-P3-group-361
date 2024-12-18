import pyrealsense2 as rs
import numpy as np
import cv2
import time
from coneDetection_modules import *
import gpiozero as gpio
import os
import psutil



# SERVO: GPIO4 = pin 7
# MotorDriver hvid: GPIO13 = 33
# MotorDriver Grå: GPIO12 = 32

servo = gpio.AngularServo(4)
motor = gpio.Motor(12, 13)

test_ID = '0' # c = RGB; d = depth

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

IR = gpio.Button(23)
LED = gpio.LED(24)

IRCount = 2
time.sleep(2)
while True:
    while IRCount % 2 != 0:
        LED.off()
        
        #framerate_array = np.array([])
        motor.forward(0.3)
        time.sleep(0.6)
        motor.forward(0.06)

        lap = 0.5 # lap = 1 means 1st lap is complete; lap = 0.5 means 1st lap is underway

        # How many laps should the av do?
        while lap <= 30:
            time_start = time.perf_counter()
            #process = psutil.Process()
            #print(process.memory_info().rss)

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
            img_RGB = np.array(color_image) #np.zeros_like(color_image, dtype=np.uint8)
            #img_RGB = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            img_RGB = img_RGB[img_RGB.shape[0]//2:,:] # slice image go get region of interest

            Depth_image_scaled = np.clip(Depth_image, 0, 5000)  # Clip to 1000mm (or any max depth you expect)
            Depth_image_normalized = np.array((Depth_image_scaled / 5000.0 * 255).astype(np.uint8))
            Depth_image_normalized = Depth_image_normalized[Depth_image_normalized.shape[0]//2:,:]
            
            #Depth_colormap = cv2.applyColorMap(Depth_image_normalized, cv2.COLORMAP_JET)

            img_cones, val_error, img_hxs_blue, img_hxs_yellow, img_hxs_orange = depth_coneDectectionC(img_RGB, Depth_image_normalized)

            # Set servo angle - the direction of the av
            #servo.source = 127 + val_error * 127/960)
            servo.angle = -val_error * 60/(img_RGB.shape[1]/2) 
            #motor.forward(0.1 - abs(val_error/(img_RGB.shape[1]/2))*0.066)
            motor.forward(0.06)
            #print(0.1 - abs(val_error/(img_RGB.shape[1]/2))*0.066)

            #motor.forward(0.03 + abs(val_error/90)*0.03)

            #print("Go")
            #print(IRCount)
            #print(IR)
            if IR.is_active == True:
                #IRCount += 1
                #motor.backward(0.1)
                motor.stop()
                #time.sleep(0.2)
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            lap_endsection = np.sum(img_hxs_orange) / (img_hxs_orange.shape[0] * img_hxs_orange.shape[1])

            # if the av has completed a lap
            if lap_endsection < 1 and int(lap*2)%2 == 0:
                lap += 0.5
            
            # if the av sees the orange cones - is about to complete a lap
            if lap_endsection > 2 and int(lap*2)%2 == 1:
                lap += 0.5

            time_end = time.perf_counter()
            #framerate_array = np.append(framerate_array, time_end-time_start)
            #print(np.average(framerate_array))
            #print(int(1/(time_end-time_start)))
            #print(-val_error * 60/(img_RGB.shape[1]/2), lap)
            #print(lap)
            framerate_array = time_end - time_start
            # Put framerate spanning 50 frames on RGB-image
            img_RGB = cv2.putText(img_RGB, 
                                        str(int(1/framerate_array)) + ' fps', (0, 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 
                                        0.3, (0, 255, 0), 1, cv2.LINE_AA)
            """Depth_image_normalized = cv2.putText(Depth_image_normalized, 
                                                    str(int(1/framerate_array)) + ' fps', (0, 10), 
                                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                                    0.3, (0, 255, 0), 1, cv2.LINE_AA)"""
            img_cones = cv2.putText(img_cones, 
                                    str(int(1/framerate_array)) + ' fps', (0, 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.3, (0, 255, 0), 1, cv2.LINE_AA)
            """img_hxs_orange = cv2.putText(img_hxs_orange, 
                                    str(int(1/framerate_array)) + ' fps', (0, 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.3, (0, 255, 0), 1, cv2.LINE_AA)"""

            # Show images
            #cv2.imshow('RAW', img_RGB)
            #cv2.imshow('Depth', Depth_image_normalized)
            #cv2.imshow('Depth', img_hxs_orange)
            #cv2.imshow('Cones', img_cones)

            # Save images
            clock = time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + "_" + str(time.time_ns() // 1000000)[9:13]
            #folder = "image_test23/" # "media/gruppe361/ESD-USB/imageTest1/"
            #os.write(folder + "RGB_" + clock + ".png", img_RGB)
            #cv2.imwrite(folder + "RGB_" + clock + ".png", img_RGB)
            #cv2.imwrite(folder + "depth_" + clock + ".png", Depth_image_normalized)
            #cv2.imwrite(folder + "orange_" + clock + ".png", img_hxs_orange)
            #cv2.imwrite(folder + "cones_" + clock + ".png", img_cones)
            #print(servo.angle)

        motor.backward(0.1)
        time.sleep(0.4)
        motor.stop()
        IRCount += 1
        servo.angle = 0
        LED.on()
        time.sleep(5)

    while IRCount % 2 == 0:
        #print("stop")
        #print(IRCount)
        motor.stop()
        servo.angle = 0
        LED.on()
       # print(servo.angle)
        #time.sleep(2)
        #print(IR)
        if IR.is_active == True:
            IRCount += 1
        

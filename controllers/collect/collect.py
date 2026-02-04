from controller import Robot, Camera
import cv2
import numpy as np
import os
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

cam = robot.getDevice("camera")
cam.enable(timestep)
width = cam.getWidth()
height = cam.getHeight()

# Create directory for images
os.makedirs("training_images", exist_ok=True)

image_count = 0
capture_interval = 50  # Capture every 50 steps

print("Starting image collection. Press Ctrl+C to stop.")

while robot.step(timestep) != -1:
    if image_count % capture_interval == 0:
        img = cam.getImage()
        if img is not None:
            img_np = np.frombuffer(img, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
            
            filename = f"training_images/img_{image_count:04d}.jpg"
            cv2.imwrite(filename, img_bgr)
            print(f"Saved: {filename}")
    
    image_count += 1

from controller import Robot, Camera, DistanceSensor, Emitter
from ultralytics import YOLO
import cv2
import numpy as np
import math
import os


robot = Robot()
timestep = int(robot.getBasicTimeStep())

cam = robot.getDevice("camera")
cam.enable(timestep)
cam.recognitionEnable(timestep)
width = cam.getWidth()
height = cam.getHeight()

ds = robot.getDevice("distance_sensor")
ds.enable(timestep)

emitter = robot.getDevice("cam_emitter")
emitter.setChannel(1)
emitter.setRange(10.0)  

model_path = os.path.join(os.path.dirname(__file__), "best_new.pt")

print("Controller directory:", os.path.dirname(__file__))
print("Files in controller directory:", os.listdir(os.path.dirname(__file__)))
model = YOLO(model_path)
print("Modèle YOLO chargé:", model_path)

object_detected = False   
last_msg = ""

while robot.step(timestep) != -1:

    distance = ds.getValue()
    threshold = 800 

    if distance < threshold:
        if not object_detected:
            print("Objet détecté par le capteur - analyse YOLO activée")
            object_detected = True

            img = cam.getImage()
            if img is None:
                continue

            img_np = np.frombuffer(img, np.uint8).reshape((height, width, 4))
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)

            results = model(img_bgr)

            r = results[0]
            if hasattr(r, 'obb') and r.obb is not None and len(r.obb.xywhr) > 0:
                ycam, xcam, w, h, theta = r.obb.xywhr[0].cpu().numpy()
            
                angle_deg = math.degrees(theta)
                if h >= w:
                    consistent_angle = angle_deg
                else:
                    consistent_angle = (angle_deg + 90) % 180 
                if consistent_angle > 90:
                    consistent_angle -= 180
            
                x = 0.000646 * xcam + 0.7681
                y = 0.000554 * ycam - 1.168
                z = 0.0
            
                data_matrix = np.array([x-0.84, y, z, math.radians(consistent_angle)])
            
                msg = ",".join([f"{v:.3f}" for v in data_matrix])
                if msg != last_msg:
                    emitter.send(msg.encode('utf-8'))
                    last_msg = msg
            
                print(f"Position: x={x:.3f}, y={y:.3f}, Taille: w={w:.1f}, h={h:.1f}, "
                      f"YOLO angle={angle_deg:.1f}°, consistent_angle={consistent_angle:.1f}°")
                print(f"Matrice_emitter: {data_matrix}")
            else:
                print("Aucun objet détecté par YOLO")

    else:
        if object_detected:
            object_detected = False

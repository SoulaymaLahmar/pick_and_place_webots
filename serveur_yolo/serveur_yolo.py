# serveur_yolo.py : capture N images depuis Webots
import socket, struct, cv2, numpy as np
import os
from ultralytics import YOLO
import math
from controller import Emitter

HOST = "0.0.0.0"
PORT = 5001

N = 2  # nombre d'images à capturer
i = 0

model = YOLO("best.pt")  # ton modèle entraîné
emitter = Emitter("yolo_emitter")
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)
print(f"Serveur de capture en attente sur {HOST}:{PORT} ...")
conn, addr = server.accept()
print("Client connecté :", addr)

try:
    while True:
        header = conn.recv(4)
        if not header:
            break
        (img_size,) = struct.unpack("!I", header)

        data = b""
        while len(data) < img_size:
            packet = conn.recv(min(4096, img_size - len(data)))
            if not packet:
                break
            data += packet
        if len(data) != img_size:
            break

        arr = np.frombuffer(data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        os.makedirs("captures_webots", exist_ok=True)
        filename = os.path.join("captures_webots", f"webots_{i:03d}.jpg")
        cv2.imwrite(filename, frame)
        print("Image sauvée :", filename)

        i += 1
        if i >= N:
            print("Capture terminée.")
            break
finally:
    conn.close()
    server.close()
    cv2.destroyAllWindows()

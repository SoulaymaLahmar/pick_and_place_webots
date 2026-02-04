from ultralytics import YOLO
import cv2

model = YOLO(r"runs/detect/train3/weights/best.pt")  # ou le chemin où est ton best.pt

img = cv2.imread("test_webots.jpg")
results = model(img)

annotated = results[0].plot()
cv2.imshow("Test Webots", annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()

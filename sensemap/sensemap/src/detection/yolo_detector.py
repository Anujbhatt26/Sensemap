import cv2
import torch
import numpy as np
from PIL import Image

class YOLODetector:
    def __init__(self, model_path, conf_threshold=0.5):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.to(self.device)
        self.conf_threshold = conf_threshold

    def detect(self, image):
        results = self.model(image)
        detections = results.pandas().xyxy[0]
        return detections[detections['confidence'] >= self.conf_threshold]

if __name__ == "__main__":
    detector = YOLODetector(model_path='path/to/yolo_model.pt')
    image = cv2.imread('path/to/image.jpg')
    detections = detector.detect(image)
    print(detections)
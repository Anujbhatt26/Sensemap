import cv2
import numpy as np

class ObjectTracker:
    def __init__(self, tracker_type='KCF'):
        self.tracker = self.create_tracker(tracker_type)

    def create_tracker(self, tracker_type):
        if tracker_type == 'KCF':
            return cv2.TrackerKCF_create()
        elif tracker_type == 'MIL':
            return cv2.TrackerMIL_create()
        elif tracker_type == 'CSRT':
            return cv2.TrackerCSRT_create()
        else:
            raise ValueError("Unsupported tracker type")

    def initialize(self, frame, bbox):
        self.tracker.init(frame, bbox)

    def update(self, frame):
        success, bbox = self.tracker.update(frame)
        if success:
            return bbox
        else:
            return None

    def draw_bbox(self, frame, bbox):
        if bbox is not None:
            (x, y, w, h) = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
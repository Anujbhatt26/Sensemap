import cv2

class Camera:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.capture = cv2.VideoCapture(self.camera_index)

    def read_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            raise Exception("Could not read frame from camera.")
        return frame

    def release(self):
        self.capture.release()

    def __del__(self):
        self.release()
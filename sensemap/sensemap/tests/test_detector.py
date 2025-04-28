import unittest
from src.detection.yolo_detector import YOLODetector

class TestYOLODetector(unittest.TestCase):
    
    def setUp(self):
        self.detector = YOLODetector()

    def test_initialization(self):
        self.assertIsNotNone(self.detector)

    def test_detection(self):
        test_image = "path/to/test/image.jpg"  # Replace with actual test image path
        detections = self.detector.detect(test_image)
        self.assertIsInstance(detections, list)
        self.assertGreater(len(detections), 0)

if __name__ == '__main__':
    unittest.main()
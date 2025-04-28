import numpy as np
import cv2
from typing import List, Dict

class SemanticMap:
    def __init__(self):
        self.map_data = {}
    
    def add_object(self, object_id: int, position: np.ndarray, label: str):
        self.map_data[object_id] = {
            'position': position,
            'label': label
        }
    
    def remove_object(self, object_id: int):
        if object_id in self.map_data:
            del self.map_data[object_id]
    
    def get_objects(self) -> Dict[int, Dict[str, np.ndarray]]:
        return self.map_data
    
    def visualize_map(self, image: np.ndarray):
        for obj_id, obj_info in self.map_data.items():
            position = obj_info['position']
            label = obj_info['label']
            cv2.circle(image, (int(position[0]), int(position[1])), 5, (0, 255, 0), -1)
            cv2.putText(image, label, (int(position[0]), int(position[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return image
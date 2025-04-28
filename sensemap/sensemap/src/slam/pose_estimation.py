import numpy as np
import cv2

class PoseEstimation:
    def __init__(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def estimate_pose(self, object_points, image_points):
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coeffs
        )
        if not success:
            raise ValueError("Pose estimation failed")
        return rotation_vector, translation_vector

    def project_points(self, object_points, rotation_vector, translation_vector):
        image_points, _ = cv2.projectPoints(
            object_points, rotation_vector, translation_vector, self.camera_matrix, self.dist_coeffs
        )
        return image_points
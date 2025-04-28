import pcl
import numpy as np

class PointCloudProcessor:
    def __init__(self):
        self.point_cloud = None

    def load_point_cloud(self, file_path):
        self.point_cloud = pcl.load(file_path)

    def filter_point_cloud(self, leaf_size=0.01):
        if self.point_cloud is None:
            raise ValueError("Point cloud not loaded.")
        
        # Create a voxel grid filter
        voxel_filter = self.point_cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
        self.point_cloud = voxel_filter.filter()

    def segment_plane(self, distance_threshold=0.01):
        if self.point_cloud is None:
            raise ValueError("Point cloud not loaded.")
        
        # Create the segmentation object
        seg = self.point_cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(distance_threshold)

        # Call segment function to obtain set of inlier indices and model coefficients
        inliers, coefficients = seg.segment()

        return inliers, coefficients

    def extract_inliers(self, inliers):
        if self.point_cloud is None:
            raise ValueError("Point cloud not loaded.")
        
        # Extract the inliers
        extracted_inliers = self.point_cloud.extract(inliers, negative=False)
        return extracted_inliers

    def extract_outliers(self, inliers):
        if self.point_cloud is None:
            raise ValueError("Point cloud not loaded.")
        
        # Extract the outliers
        extracted_outliers = self.point_cloud.extract(inliers, negative=True)
        return extracted_outliers
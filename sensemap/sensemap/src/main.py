#!/usr/bin/env python3
import yaml
import rospy
from utils.camera import Camera
from slam.orb_slam import ORB_SLAM
from detection.yolo_detector import YOLODetector
from mapping.semantic_map import SemanticMap

def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rospy.init_node('sensemap_node')
    
    # Load configurations
    camera_config = load_config(rospy.get_param('~camera_params'))
    yolo_config = load_config(rospy.get_param('~yolo_config'))
    
    # Initialize components
    camera = Camera()
    slam = ORB_SLAM(camera_config['camera']['intrinsic'])
    detector = YOLODetector(yolo_config['model']['path'], 
                          yolo_config['model']['conf_threshold'])
    semantic_map = SemanticMap()
    
    rate = rospy.Rate(camera_config['camera']['fps'])
    
    while not rospy.is_shutdown():
        try:
            # Get camera frame
            frame = camera.read_frame()
            
            # Process SLAM
            keypoints, descriptors = slam.process_frame(frame)
            
            # Detect objects
            detections = detector.detect(frame)
            
            # Update semantic map
            for i, detection in detections.iterrows():
                semantic_map.add_object(
                    i,
                    position=detection[['xmin', 'ymin']].values,
                    label=detection['name']
                )
            
            # Visualize results (can be enhanced using visualization.py)
            semantic_map.visualize_map(frame)
            
            rate.sleep()
            
        except rospy.ROSInterruptException:
            break
        
    camera.release()

if __name__ == '__main__':
    main()
"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rclpy
import yaml
from sensor_msgs.msg import PointCloud2
from rclpy import  qos
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge


class lidar_publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        

        self.publisher_lidar = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.subscription = self.create_subscription(PointCloud2, '/ouster/points',self.listener_callback,qos.qos_profile_sensor_data)
        self.lidar_msg = PointCloud2()
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):

        current_time = self.get_clock().now()
        
        self.lidar_msg = msg

        self.lidar_msg.header.stamp.sec = int(current_time.nanoseconds // 1e9)
        self.lidar_msg.header.stamp.nanosec = int(current_time.nanoseconds // 1e9)
        
        self.publisher_lidar.publish(self.lidar_msg)
 
        


def main(args=None):
    rclpy.init(args=args)
    publisher = lidar_publisher()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    

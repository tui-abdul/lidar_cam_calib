import rclpy
import yaml
from sensor_msgs.msg import Image
from rclpy import  qos
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge


class camera_publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        

        self.publisher_camera = self.create_publisher(Image, '/image_raw', 10)
        self.subscription = self.create_subscription(Image, '/cam_teleoperation/front_mid',self.listener_callback,10)
        self.camera_msg = Image()
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):

        current_time = self.get_clock().now()
        
        self.camera_msg = msg

        self.camera_msg.header.stamp.sec = int(current_time.nanoseconds // 1e9)
        self.camera_msg.header.stamp.nanosec = int(current_time.nanoseconds // 1e9)
        
        self.publisher_camera.publish(self.camera_msg)
 
        


def main(args=None):
    rclpy.init(args=args)
    publisher = camera_publisher()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
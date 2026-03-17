#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('rate', 10.0)
        
        # Get parameters
        yaml_file = self.get_parameter('camera_info_file').value
        topic = self.get_parameter('camera_info_topic').value
        frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value
        
        # Load camera info from YAML
        with open(yaml_file, 'r') as f:
            calib_data = yaml.safe_load(f)
        
        # Create CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = frame_id
        self.camera_info_msg.height = calib_data['image_height']
        self.camera_info_msg.width = calib_data['image_width']
        self.camera_info_msg.distortion_model = calib_data['distortion_model']
        self.camera_info_msg.d = calib_data['distortion_coefficients']['data']
        self.camera_info_msg.k = calib_data['camera_matrix']['data']
        self.camera_info_msg.r = calib_data['rectification_matrix']['data']
        self.camera_info_msg.p = calib_data['projection_matrix']['data']
        
        # Create publisher
        self.publisher = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
        self.get_logger().info(f'Publishing camera info to {topic} at {rate} Hz')
    
    def timer_callback(self):
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
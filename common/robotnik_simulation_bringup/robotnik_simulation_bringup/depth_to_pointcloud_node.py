#!/usr/bin/env python3
"""
Converts depth images to PointCloud2 messages.
Subscribes to depth/image_raw and depth/camera_info and publishes depth/points.
Uses ROS_NAMESPACE environment variable to determine the namespace to run in.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.depth_image = None
        self.camera_info = None
        
        # Subscriptions - using relative namespace
        self.depth_sub = self.create_subscription(
            Image,
            'depth/image_raw',
            self.depth_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            'depth/camera_info',
            self.info_callback,
            10
        )
        
        # Publisher - using relative namespace
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            'depth/points',
            10
        )
        
        self.get_logger().info(f'Depth to PointCloud converter node initialized in namespace: {self.get_namespace()}')
        
    def info_callback(self, msg: CameraInfo):
        """Store camera info parameters."""
        self.camera_info = msg
        
    def depth_callback(self, msg: Image):
        """Convert depth image to pointcloud."""
        if self.camera_info is None:
            return
            
        try:
            cloud = self.convert_depth_to_cloud(msg)
            self.cloud_pub.publish(cloud)
        except Exception as e:
            self.get_logger().error(f'Error converting depth to pointcloud: {e}')
    
    def convert_depth_to_cloud(self, depth_msg: Image) -> PointCloud2:
        """Convert depth image to PointCloud2."""
        # Decode depth image directly to avoid cv_bridge/cv2 runtime dependency.
        if depth_msg.encoding in ('16UC1', 'mono16'):
            dtype = np.uint16
            scale_to_meters = 0.001
        elif depth_msg.encoding == '32FC1':
            dtype = np.float32
            scale_to_meters = 1.0
        else:
            raise ValueError(f'Unsupported depth encoding: {depth_msg.encoding}')

        depth_image = np.frombuffer(depth_msg.data, dtype=dtype).reshape((depth_msg.height, depth_msg.width)).astype(np.float32)
        depth_image *= scale_to_meters
        
        # Get camera parameters
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        
        height, width = depth_image.shape
        points = []
        
        # Create point cloud from depth image
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]
                
                # Filter invalid points
                if z > 0 and np.isfinite(z):
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        
        # Handle empty point cloud
        if len(points) == 0:
            points = [[0.0, 0.0, 0.0]]
        
        points_array = np.array(points, dtype=np.float32)
        
        # Define PointCloud2 fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header = depth_msg.header
        cloud.height = 1
        cloud.width = len(points_array)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12  # 3 floats * 4 bytes
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points_array.tobytes()
        
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


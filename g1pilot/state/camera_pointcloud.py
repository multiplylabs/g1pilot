#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import image_encodings

class DepthToCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_cloud_py')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('cloud_topic', 'depth/points')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('max_range_m', 10.0)
        self.declare_parameter('publish_organized', True)
        self.declare_parameter('use_nan_for_invalid', True)

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.info_topic  = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.max_range   = float(self.get_parameter('max_range_m').value)
        self.pub_org     = bool(self.get_parameter('publish_organized').value)
        self.use_nan     = bool(self.get_parameter('use_nan_for_invalid').value)

        self.have_info = False
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.width = self.height = 0
        self._u_factor = None
        self._v_factor = None
        self._xyz_buf = None

        qos = qos_profile_sensor_data
        self.sub_info  = self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos)
        self.sub_depth = self.create_subscription(Image,      self.depth_topic, self._on_depth, qos)
        self.pub_cloud = self.create_publisher(PointCloud2, self.cloud_topic, qos)

        self.get_logger().info(f"DepthToCloud PY → depth: {self.depth_topic} | info: {self.info_topic} | out: {self.cloud_topic}")

    # ---------------- CameraInfo ----------------
    def _on_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0]); self.fy = float(msg.k[4])
        self.cx = float(msg.k[2]); self.cy = float(msg.k[5])
        self.width  = int(msg.width)
        self.height = int(msg.height)
        self.have_info = (self.fx > 0.0 and self.fy > 0.0 and self.width > 1 and self.height > 1)
        if not self.have_info:
            self.get_logger().warn("Invalid CameraInfo. Waiting for correct intrinsics...")
            return
        self._prepare_grids()

    def _prepare_grids(self):
        u = np.arange(self.width, dtype=np.float32)
        v = np.arange(self.height, dtype=np.float32)
        self._u_factor = (u - self.cx) / self.fx
        self._v_factor = (v - self.cy) / self.fy
        self._xyz_buf = np.empty((self.height, self.width, 3), dtype=np.float32)

    def _on_depth(self, img: Image):
        if not self.have_info:
            self.get_logger().warn_throttle(2.0, "No CameraInfo yet. Cloud not published.")
            return

        H, W = int(img.height), int(img.width)
        if (H != self.height) or (W != self.width) or (self._u_factor is None):
            self.width, self.height = W, H
            self._prepare_grids()

        if img.encoding in (image_encodings.TYPE_16UC1, image_encodings.MONO16):
            z = np.frombuffer(img.data, dtype=np.uint16).reshape(H, W).astype(np.float32) * self.depth_scale
            invalid = (z <= 0.0)
        elif img.encoding == image_encodings.TYPE_32FC1:
            z = np.frombuffer(img.data, dtype=np.float32).reshape(H, W)
            if self.depth_scale != 1.0:
                z = z * self.depth_scale
            invalid = np.isnan(z) | (z <= 0.0)
        else:
            self.get_logger().warn_throttle(5.0, f"Encoding not supported: {img.encoding}")
            return

        if self.max_range and self.max_range > 0.0:
            invalid = invalid | (z > self.max_range)

        X = z * self._u_factor[None, :]
        Y = z * self._v_factor[:, None]

        cloud = PointCloud2()
        cloud.header = img.header
        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12 

        if self.pub_org:
            xyz = self._xyz_buf
            xyz[..., 0] = X
            xyz[..., 1] = Y
            xyz[..., 2] = z

            if self.use_nan:
                if np.any(invalid):
                    xyz[invalid, 0] = np.nan
                    xyz[invalid, 1] = np.nan
                    xyz[invalid, 2] = np.nan
                cloud.is_dense = False
            else:
                if np.any(invalid):
                    xyz[invalid, :] = 0.0
                cloud.is_dense = True

            cloud.height = H
            cloud.width  = W
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = memoryview(xyz.tobytes(order='C'))
        else:
            valid = ~invalid
            n = int(valid.sum())
            if n == 0:
                cloud.height = 1
                cloud.width  = 0
                cloud.row_step = 0
                cloud.data = b''
                cloud.is_dense = True
                self.pub_cloud.publish(cloud)
                return

            xyz = np.empty((n, 3), dtype=np.float32)
            xyz[:, 0] = X[valid]
            xyz[:, 1] = Y[valid]
            xyz[:, 2] = z[valid]

            cloud.height = 1
            cloud.width  = n
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = memoryview(xyz.tobytes(order='C'))
            cloud.is_dense = True

        self.pub_cloud.publish(cloud)

def main():
    rclpy.init()
    node = DepthToCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

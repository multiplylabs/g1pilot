#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs_py import point_cloud2


class DepthToColoredPC(Node):
    """
    Convierte depth + color en PointCloud2 con RGB.
    Asume que la imagen de profundidad está registrada al frame de la cámara color.
    """

    def __init__(self):
        super().__init__('depth_to_colored_pointcloud')

        # Parámetros
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.05)  # tolerancia en la sincronización (s)
        self.declare_parameter('frame_id', '')  # si vacío usa el de la imagen color

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        self.frame_id_param = self.get_parameter('frame_id').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.cam_info: CameraInfo | None = None

        # Suscripción a CameraInfo (solo necesitamos la de color si depth está registrada a color)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)

        # Sincronización aproximada de depth y color
        self.sub_depth = Subscriber(self, Image, depth_topic)
        self.sub_color = Subscriber(self, Image, color_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_color], queue_size=queue_size, slop=slop
        )
        self.ts.registerCallback(self._sync_cb)

        # Publicador de PointCloud2
        self.pub = self.create_publisher(PointCloud2, 'colored_pointcloud', 1)

        self.get_logger().info(
            f'✅ depth_to_colored_pointcloud listo.\n'
            f'   depth: {depth_topic}\n'
            f'   color: {color_topic}\n'
            f'   info : {info_topic}\n'
            f'   out  : colored_pointcloud'
        )

    def _info_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def _sync_cb(self, depth_msg: Image, color_msg: Image):
        if self.cam_info is None:
            self.get_logger().warn('Esperando CameraInfo...')
            return

        # Obtén intrínsecos
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        # Depth a numpy
        try:
            # depth puede venir como 16UC1 (milímetros) o 32FC1 (metros)
            if depth_msg.encoding in ('16UC1', 'mono16', 'uint16'):
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
                depth *= 0.001  # mm -> m
            else:
                # Asumimos metros
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo depth: {e}')
            return

        # Color a numpy (RGB8)
        try:
            # Si viene en BGR8, lo pasamos a RGB
            if color_msg.encoding == 'bgr8':
                bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
                rgb = bgr[..., ::-1].copy()
            else:
                rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo color: {e}')
            return

        h, w = depth.shape
        if rgb.shape[0] != h or rgb.shape[1] != w:
            self.get_logger().warn(
                f'Tamaño depth ({w}x{h}) != color ({rgb.shape[1]}x{rgb.shape[0]}). '
                f'Se recortará al mínimo común.'
            )
            H = min(h, rgb.shape[0])
            W = min(w, rgb.shape[1])
            depth = depth[:H, :W]
            rgb = rgb[:H, :W]
            h, w = H, W

        # Malla de pixeles
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        # Máscara válida
        valid = np.isfinite(depth) & (depth > 0.0)

        if not np.any(valid):
            self.get_logger().warn('No hay valores válidos de profundidad.')
            return

        z = depth[valid]
        x = (uu[valid] - cx) * z / fx
        y = (vv[valid] - cy) * z / fy

        # Colores (uint8)
        colors = rgb[valid]  # shape [N, 3] en RGB
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)

        # Empaquetar RGB a un uint32 estilo PCL (0x00RRGGBB) y verlo como float32
        rgb_uint32 = (r << 16) | (g << 8) | b
        rgb_float = rgb_uint32.view(np.float32)

        # Armar lista de puntos (x, y, z, rgb)
        points = np.column_stack((x, y, z, rgb_float))

        # Crear PointCloud2
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header_frame = self.frame_id_param if self.frame_id_param else (color_msg.header.frame_id or 'camera_color_optical_frame')
        header = depth_msg.header  # tiempo del depth
        header.frame_id = header_frame

        cloud_msg = point_cloud2.create_cloud(header, fields, points.tolist())

        self.pub.publish(cloud_msg)


def main():
    rclpy.init()
    node = DepthToColoredPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

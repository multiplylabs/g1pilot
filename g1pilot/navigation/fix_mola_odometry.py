#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return (x, y, z, w)


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (0.0, 0.0, 0.0, 1.0) if n == 0.0 else (x / n, y / n, z / n, w / n)


def rx_pi_apply_to_vec3(x, y, z):
    return (x, -y, -z)


def rx_pi_quat():
    return euler_to_quat(math.pi, 0.0, 0.0)


class OdomReorienter(Node):
    def __init__(self):
        super().__init__('odom_reorienter')

        self.declare_parameter('in_topic', '/lidar_odometry/pose')
        self.declare_parameter('out_topic', '/lidar_odometry/pose_fixed')
        self.declare_parameter('roll_offset_deg', 0.0)
        self.declare_parameter('pitch_offset_deg', 0.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('normalize_quaternion', True)
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('fix_left_right_inversion', True)

        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        roll_deg = self.get_parameter('roll_offset_deg').get_parameter_value().double_value
        pitch_deg = self.get_parameter('pitch_offset_deg').get_parameter_value().double_value
        yaw_deg = self.get_parameter('yaw_offset_deg').get_parameter_value().double_value
        self.normalize_quat = self.get_parameter('normalize_quaternion').get_parameter_value().bool_value
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value
        self.fix_lr = self.get_parameter('fix_left_right_inversion').get_parameter_value().bool_value

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        q_user = euler_to_quat(roll, pitch, yaw)

        q_fix = rx_pi_quat() if self.fix_lr else (0.0, 0.0, 0.0, 1.0)

        self.q_prefix = quat_multiply(q_user, q_fix)

        self.sub = self.create_subscription(Odometry, in_topic, self.cb_odom, 10)
        self.pub = self.create_publisher(Odometry, out_topic, 10)

    def cb_odom(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override

        px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        if self.fix_lr:
            px, py, pz = rx_pi_apply_to_vec3(px, py, pz)
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz

        q_in = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        q_out = quat_multiply(self.q_prefix, q_in)
        if self.normalize_quat:
            q_out = quat_normalize(q_out)
        out.pose.pose.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])

        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist

        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomReorienter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import pinocchio as pin
from pinocchio import SE3

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)
from g1pilot.utils.helpers import mat_to_quat_wxyz, quat_wxyz_to_matrix, quat_slerp
from g1pilot.utils.ik_solver import G1IKSolver


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms using the smooth IK solver.

    Publishes the current joint states for RViz or simulation and
    updates joint targets based on incoming end-effector goals.
    """

    def __init__(self):
        super().__init__("arm_controller")

        self.use_robot = False
        self.frame = "pelvis"
        self.arm_velocity_limit = 2.0  # rad/s
        self._last_q_target = None

        # Publishers / Subscribers
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_solver = G1IKSolver(debug=False)
        self.create_subscription(PoseStamped, "/g1pilot/right_hand_goal", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/g1pilot/left_hand_goal", self._left_goal_callback, 10)

        self.get_logger().info("ArmController initialized (simulation mode).")

    # ------------------------------------------------------------------
    # Core publishing loop
    # ------------------------------------------------------------------

    def publish_joint_state(self):
        """Publishes smooth, velocity-limited JointState for RViz visualization."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
        js.position = [0.0] * len(js.name)

        # Initialize if first time
        if not hasattr(self, "_last_q_target"):
            self._last_q_target = np.zeros(14, dtype=float)

        # --- Base targets (zeros) ---
        q_target = np.zeros(14, dtype=float)

        # Get IK solutions if any
        current_all = np.zeros(29, dtype=float)
        q_dict = self.ik_solver.get_joint_targets(current_all)

        if "left" in q_dict:
            q_target[0:7] = q_dict["left"]
        if "right" in q_dict:
            q_target[7:14] = q_dict["right"]

        # --- Velocity limiting (smoothness) ---
        dt = 0.05  # 20 Hz
        max_step = self.arm_velocity_limit * dt

        if not hasattr(self, "_last_q_target") or self._last_q_target is None or not isinstance(self._last_q_target, np.ndarray):
            self._last_q_target = np.zeros_like(q_target)

        dq = np.asarray(q_target, dtype=float) - np.asarray(self._last_q_target, dtype=float)
        dq = np.clip(dq, -max_step, max_step)
        q_smooth = self._last_q_target + dq
        self._last_q_target = q_smooth.copy()


        # --- Map arm joints into full 29-joint array ---
        for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[idx])
        for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[7 + idx])

        # --- Publish always (even if zeros) ---
        self.joint_pub.publish(js)


    # ------------------------------------------------------------------
    # Pose callbacks
    # ------------------------------------------------------------------

    def _right_goal_callback(self, msg: PoseStamped):
        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal = SE3(q.matrix(), np.array([p.x, p.y, p.z]))
        self.ik_solver.set_goal("right", T_goal)

    def _left_goal_callback(self, msg: PoseStamped):
        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal = SE3(q.matrix(), np.array([p.x, p.y, p.z]))
        self.ik_solver.set_goal("left", T_goal)

    def _transform_pose_to_world(self, ps):
        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return ps


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

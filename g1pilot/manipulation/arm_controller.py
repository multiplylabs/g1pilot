#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np
import rclpy
from rclpy.node import Node
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
from g1pilot.utils.ik_solver import G1IKSolver


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms (simulation only).
    - Publishes JointState to /joint_states.
    - Moves both arms to predefined 'home' pose when /g1pilot/homming_arms=True.
    - Holds that pose until a new goal is received from either arm topic.
    """

    def __init__(self):
        super().__init__("arm_controller_sim")

        # --- Parameters ---
        self.declare_parameter("arm_velocity_limit", 2.0)
        self.arm_velocity_limit = self.get_parameter("arm_velocity_limit").get_parameter_value().double_value
        self.frame = "pelvis"

        # --- Internal state ---
        self._last_q_target = np.zeros(14, dtype=float)
        self.rate_hz = 50.0
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02

        # --- IK Solver ---
        self.ik_solver = G1IKSolver(debug=False)

        # --- TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Home positions ---
        self.home_right = np.array([
            0.6604386568069458,
            -0.09250623732805252,
            0.022230736911296844,
            -0.839135468006134,
            0.10722286254167557,
            0.2716066539287567,
            0.06743379682302475,
        ])
        self.home_left = np.array([
            0.9230489730834961,
            -0.06001700088381767,
            0.03733086213469505,
            -0.7793461680412292,
            -0.06614094227552414,
            -0.11401312798261642,
            -0.29750025272369385,
        ])

        # --- Publishers and Subscribers ---
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/g1pilot/right_hand_goal", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/g1pilot/left_hand_goal", self._left_goal_callback, 10)
        self.create_subscription(Bool, "/g1pilot/arms_controlled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "/g1pilot/homming_arms", self._homming_callback, 10)

        self.get_logger().info("ArmController simulation initialized.")

        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_joint_state_loop)

    def _arms_controlled_callback(self, msg: Bool):
        """
        Subscriber callback for `/g1pilot/arms_controlled`.

        Enables or disables arm movement. When disabled, the node stops updating
        the published joint positions but keeps the current posture.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            Message indicating whether arm control is enabled (`True`) or disabled (`False`).
        """
        self.arms_enabled = msg.data
        if self.arms_enabled:
            self.get_logger().info("Arm simulation ENABLED.")
        else:
            self.get_logger().info("Arm simulation DISABLED")

    def _homming_callback(self, msg: Bool):
        """
        Subscriber callback for `/g1pilot/homming_arms`.

        Initiates the homing procedure when `msg.data` is True, commanding both
        arms toward the predefined "home" joint targets. Once the target is reached,
        motion is locked until a new pose goal is received.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            When `True`, starts the homing sequence for both arms.
        """
        if msg.data:
            self.get_logger().info("Moving both arms to HOME position.")
            self.homing_active = True
            self.homing_reached = False

    def _right_goal_callback(self, msg: PoseStamped):
        """
        Subscriber callback for `/g1pilot/right_hand_goal`.

        Sets a new pose goal for the right arm using the inverse kinematics solver.
        If the node is currently locked at the home position, the receipt of this message
        automatically unlocks motion control.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Target pose of the right arm’s end-effector.
        """
        if self.homing_active:
            return
        if self.homing_reached:
            self.homing_reached = False

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal = SE3(q.matrix(), np.array([p.x, p.y, p.z]))
        self.ik_solver.set_goal("right", T_goal)

    def _left_goal_callback(self, msg: PoseStamped):
        """
        Subscriber callback for `/g1pilot/left_hand_goal`.

        Sets a new pose goal for the left arm using the inverse kinematics solver.
        If the node is currently locked at the home position, the receipt of this message
        automatically unlocks motion control.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Target pose of the left arm’s end-effector.
        """
        if self.homing_active:
            return
        if self.homing_reached:
            self.homing_reached = False 

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal = SE3(q.matrix(), np.array([p.x, p.y, p.z]))
        self.ik_solver.set_goal("left", T_goal)

    def _transform_pose_to_world(self, ps):
        """
        Transforms an incoming PoseStamped message to the world frame if required.

        If the pose’s frame differs from `self.frame`, a TF lookup is performed
        to express the pose in the `pelvis` frame before processing.

        Parameters
        ----------
        ps : geometry_msgs.msg.PoseStamped
            Input pose possibly expressed in another reference frame.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            Transformed pose expressed in `self.frame` coordinates.
        """
        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return ps

    def _publish_joint_state_loop(self):
        """
        Main periodic control loop executed at `self.rate_hz`.

        Depending on the current mode:
        - If `homing_active=True`: moves both arms smoothly toward their home positions.
        - If `homing_reached=True`: maintains the home posture.
        - Otherwise: computes joint angles from the IK solver and publishes them.

        Smooth motion is achieved by clipping the change per iteration
        according to `self.arm_velocity_limit`.

        Publishes
        ----------
        /joint_states : sensor_msgs/JointState
            Updated joint positions for visualization or testing.
        """
        if not self.arms_enabled:
            return

        if self.homing_active:
            q_target = np.concatenate((self.home_left, self.home_right))
        elif self.homing_reached:
            q_target = np.concatenate((self.home_left, self.home_right))
        else:
            current_all = np.zeros(29, dtype=float)
            q_dict = self.ik_solver.get_joint_targets(current_all)
            q_target = np.zeros(14, dtype=float)
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

        dt = 1.0 / self.rate_hz
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)
        q_smooth = self._last_q_target + dq
        self._last_q_target = q_smooth.copy()

        if self.homing_active:
            err = np.linalg.norm(q_smooth - np.concatenate((self.home_left, self.home_right)))
            if err < self.homing_tolerance:
                self.homing_active = False
                self.homing_reached = True
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()
                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()
                self.get_logger().info("Home position reached.")

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
        js.position = [0.0] * len(js.name)
        for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[idx])
        for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[7 + idx])
        self.joint_pub.publish(js)


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

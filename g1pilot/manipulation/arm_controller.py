#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
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
import threading

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointIndex,
    DataBuffer,
)

from g1pilot.utils.helpers import mat_to_quat_wxyz, quat_wxyz_to_matrix, quat_slerp
from g1pilot.utils.ik_solver import G1IKSolver

# --- SDK imports for real robot control ---
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

class ArmController(Node):
    """
    ROS 2 node controlling G1 arms using the smooth IK solver.

    - Simulation mode: publishes JointState for RViz.
    - Robot mode: communicates via DDS to send motor commands.
    """

    def __init__(self):
        super().__init__("arm_controller")

        # --- Parameters ---
        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", "eth0")
        self.use_robot = self.get_parameter("use_robot").get_parameter_value().bool_value
        self.interface = self.get_parameter("interface").get_parameter_value().string_value

        self.frame = "pelvis"
        self.arm_velocity_limit = 2.0  # rad/s
        self._last_q_target = None
        self.rate_hz = 20.0  # update rate
        self.lowstate_buffer = DataBuffer()

        # --- IK Solver ---
        self.ik_solver = G1IKSolver(debug=False)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- ROS publishers/subscribers ---
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/g1pilot/right_hand_goal", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/g1pilot/left_hand_goal", self._left_goal_callback, 10)
        # --- Nuevo flag y subscriber ---
        self.arms_enabled = False
        self.create_subscription(Bool, "/g1pilot/arms_controlled", self._arms_controlled_callback, 10)


        # --- DDS setup if use_robot=True ---
        self.crc = None
        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.msg = None
        self.latest_state = None

        if self.use_robot:
            self._init_dds()
            self.get_logger().info(f"ArmController initialized (REAL ROBOT mode, iface={self.interface}).")
        else:
            self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_joint_state)
            self.get_logger().info("ArmController initialized (simulation mode).")


        self.control_mode = True
        self.control_dt = 1.0 / 250.0  # 250 Hz
        self.ctrl_lock = threading.Lock()
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()


    # ------------------------------------------------------------------
    # DDS initialization
    # ------------------------------------------------------------------
    def _arms_controlled_callback(self, msg: Bool):
        self.arms_enabled = msg.data
        if self.arms_enabled:
            self.get_logger().info("🟢 Arm control ENABLED.")
        else:
            self.get_logger().info("🔴 Arm control DISABLED — Holding current pose.")


    def get_mode_machine(self):
        if self.use_robot:
            msg = self.lowstate_buffer.GetData()
            return getattr(msg, "mode_machine", 0) if msg is not None else 0
        return 0
    
    def get_current_motor_q(self):
        if self.use_robot:
            msg = self.lowstate_buffer.GetData()
            return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)
        return self.sim_current_q_all.copy()
    
    def clip_arm_q_target(self, target_q, velocity_limit):
        if self._last_cmd_q is None:
            self._last_cmd_q = self.get_current_dual_arm_q().copy()
        target_q = np.asarray(target_q, dtype=float).reshape(-1)
        last = np.asarray(self._last_cmd_q, dtype=float).reshape(-1)
        dt = self._compute_dt()
        max_step = float(velocity_limit) * dt
        delta = target_q - last
        delta = np.clip(delta, -max_step, max_step)
        return last + delta

    def _hold_non_arm_joints(self):
        if not self.use_robot:
            return
        arm_vals  = {m.value for m in G1_29_JointArmIndex}
        weak_vals = {m.value for m in G1_29_JointWeakIndex}
        current_all = self.get_current_motor_q()

        self.msg.mode_pr = 0
        for jid in G1_29_JointIndex:
            if jid.value in arm_vals:
                continue
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in weak_vals:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            else:
                self.msg.motor_cmd[jid].kp = self.kp_high
                self.msg.motor_cmd[jid].kd = self.kd_high
            self.msg.motor_cmd[jid].q   = float(current_all[jid.value])
            self.msg.motor_cmd[jid].dq  = 0.0
            self.msg.motor_cmd[jid].tau = 0.0

    def _arm_set_mode(self, mode:int, kp:float=0.0, kd:float=0.0):
        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            if self.use_robot:
                self.msg.motor_cmd[jid].mode = mode
                if mode == 1:
                    if jid.value in wrist_vals:
                        self.msg.motor_cmd[jid].kp = self.kp_wrist
                        self.msg.motor_cmd[jid].kd = self.kd_wrist
                    else:
                        self.msg.motor_cmd[jid].kp = self.kp_low
                        self.msg.motor_cmd[jid].kd = self.kd_low
                else:
                    self.msg.motor_cmd[jid].kp = kp
                    self.msg.motor_cmd[jid].kd = kd

    def _ctrl_motor_state(self):
        """Control loop que envía LowCmds al robot a alta frecuencia (250 Hz)."""
        rate_hz = 250.0
        control_dt = 1.0 / rate_hz
        all_joint_names = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]

        while rclpy.ok():
            start = time.time()

            # Obtener metas de IK
            current_all = np.zeros(29, dtype=float)
            q_dict = self.ik_solver.get_joint_targets(current_all)

            q_target = np.zeros(14, dtype=float)
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

            # Suavizado de velocidad
            if not hasattr(self, "_last_cmd_q") or self._last_cmd_q is None:
                self._last_cmd_q = q_target.copy()
            dq = q_target - self._last_cmd_q
            max_step = self.arm_velocity_limit * control_dt
            dq = np.clip(dq, -max_step, max_step)
            q_smooth = self._last_cmd_q + dq
            self._last_cmd_q = q_smooth.copy()

            if self.arms_enabled:
                if self.use_robot:
                    # Asegurar torque activo
                    self.msg.mode_pr = 1
                    self.msg.mode_machine = 0

                    # Configurar comandos de motores
                    for idx, jid in enumerate(range(14)):
                        cmd = self.msg.motor_cmd[jid]
                        cmd.mode = 1
                        cmd.q = float(q_smooth[idx])
                        cmd.dq = 0.0
                        cmd.tau = 0.0
                        # kp/kd iguales al código oficial
                        if jid in [4, 5, 6, 11, 12, 13]:
                            cmd.kp = 40.0
                            cmd.kd = 1.5
                        else:
                            cmd.kp = 150.0
                            cmd.kd = 4.0

                    # Enviar al DDS
                    self.msg.crc = self.crc.Crc(self.msg)
                    self.lowcmd_publisher.Write(self.msg)

                else:
                    # Simulación: publicar JointState
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = all_joint_names
                    js.position = [0.0] * len(js.name)
                    for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
                        js.position[joint_idx] = float(q_smooth[idx])
                    for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
                        js.position[joint_idx] = float(q_smooth[7 + idx])
                    self.joint_pub.publish(js)

                # Mantener frecuencia de control
                elapsed = time.time() - start
                time.sleep(max(0.0, control_dt - elapsed))


    def _init_dds(self):
        """Initialize Unitree DDS communication."""
        self.get_logger().info(f"[DDS] Initializing DDS on {self.interface}...")
        ChannelFactoryInitialize(0, self.interface)
        time.sleep(1.0)


        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init()

        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = getattr(self.latest_state, "mode_machine", 0)
        self.all_motor_q = np.zeros(35, dtype=float)
        for jid in range(14):
            self.msg.motor_cmd[jid].mode = 1
            self.msg.motor_cmd[jid].kp = 150.0
            self.msg.motor_cmd[jid].kd = 4.0
            self.msg.motor_cmd[jid].q = 0.0

        # Esperar primer mensaje
        self.get_logger().info("[DDS] Waiting for first lowstate message...")
        for _ in range(50):
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.latest_state = msg
                break
            time.sleep(0.1)

        if self.latest_state is None:
            self.get_logger().warn("⚠️ No se recibió ningún mensaje de LowState (verifica la conexión).")
        else:
            self.get_logger().info("✅ LowState activo, DDS listo para enviar comandos.")

    # ------------------------------------------------------------------
    # Simulation mode publishing
    # ------------------------------------------------------------------
    
    def publish_joint_state(self):
        """Publishes smooth, velocity-limited JointState for RViz visualization."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
        js.position = [0.0] * len(js.name)

        if self._last_q_target is None:
            self._last_q_target = np.zeros(14, dtype=float)

        q_target = np.zeros(14, dtype=float)
        current_all = np.zeros(29, dtype=float)
        q_dict = self.ik_solver.get_joint_targets(current_all)

        if "left" in q_dict:
            q_target[0:7] = q_dict["left"]
        if "right" in q_dict:
            q_target[7:14] = q_dict["right"]

        # --- Velocity limiting ---
        dt = 1.0 / self.rate_hz
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)
        q_smooth = self._last_q_target + dq
        self._last_q_target = q_smooth.copy()

        for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[idx])
        for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
            js.position[joint_idx] = float(q_smooth[7 + idx])

        self.joint_pub.publish(js)

    # ------------------------------------------------------------------
    # Real robot publishing (DDS)
    # ------------------------------------------------------------------

    def publish_lowcmd(self):
        """Compute IK and send LowCmd to robot (mismo flujo del G1_29_ArmController)."""
        # Leer estado actual
        msg_state = self.lowstate_subscriber.Read()
        if msg_state is not None:
            self.latest_state = msg_state
        if self.latest_state is None:
            self.get_logger().warn_once("⚠️ No LowState data yet — robot might be disconnected.")
            return

        # Resolver IK
        current_all = np.zeros(29, dtype=float)
        q_dict = self.ik_solver.get_joint_targets(current_all)

        q_target = np.zeros(14, dtype=float)
        if "left" in q_dict:
            q_target[0:7] = q_dict["left"]
        if "right" in q_dict:
            q_target[7:14] = q_dict["right"]

        # Suavizado
        dt = 1.0 / self.rate_hz
        max_step = self.arm_velocity_limit * dt
        if self._last_q_target is None:
            self._last_q_target = q_target.copy()
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)
        q_smooth = self._last_q_target + dq
        self._last_q_target = q_smooth.copy()

        # --- BLOQUE IDÉNTICO AL G1_29_ArmController ---
        self.msg.mode_machine = getattr(self.latest_state, "mode_machine", 0)
        self.msg.mode_pr = 1  # enable control

        try:
            # activador dummy para handshake con firmware
            self.msg.motor_cmd[0].q = 1.0
        except Exception:
            pass

        for idx, jid in enumerate(range(14)):  # primeros 14 joints (brazos)
            self.msg.motor_cmd[jid].mode = 1
            self.msg.motor_cmd[jid].q   = float(q_smooth[idx])
            self.msg.motor_cmd[jid].dq  = 0.0
            self.msg.motor_cmd[jid].tau = 0.0
            # Ganancias igual que G1_29
            if jid in [4, 5, 6, 11, 12, 13]:  # muñeca
                self.msg.motor_cmd[jid].kp = 40.0
                self.msg.motor_cmd[jid].kd = 1.5
            else:
                self.msg.motor_cmd[jid].kp = 150.0
                self.msg.motor_cmd[jid].kd = 4.0

        # Checksum y envío
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)

        # Debug cada 5 s
        if self.get_clock().now().nanoseconds % int(5e9) < 5e7:
            self.get_logger().info(f"[DDS] Sent q0={q_smooth[0]:.3f}, q1={q_smooth[1]:.3f}, crc={self.msg.crc}")


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

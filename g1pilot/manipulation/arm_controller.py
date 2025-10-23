#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import pinocchio as pin
from pinocchio import SE3

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)

from g1pilot.utils.ik_solver import G1IKSolver

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointIndex,
    DataBuffer,
)


def _yaw_from_R(R: np.ndarray) -> float:
    """Yaw (Z) desde matriz de rotación."""
    return math.atan2(R[1, 0], R[0, 0])


def _mat_to_quat_wxyz(R: np.ndarray):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z])


def _quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    return pin.Quaternion(w, x, y, z).matrix()


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms with external IK (G1IKSolver) and Unitree DDS.
    - IK goal low-pass + orientation step limiting
    - EE auto-calibration (per-side) with static offsets
    - Velocity clipping and non-arm joint holding
    - Homing that re-seeds IK goals on completion
    """

    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Arm Controller Node started.")

        # ============ Parámetros ============
        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", "eth0")
        self.declare_parameter("arm_velocity_limit", 2.0)
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("ik_world_frame", "pelvis")
        self.declare_parameter("ik_alpha", 0.2)
        self.declare_parameter("ik_goal_filter_alpha", 0.25)
        self.declare_parameter("ik_orientation_mode", "full")  # full|yaw|none
        self.declare_parameter("ik_max_ori_step_rad", 0.35)
        self.declare_parameter("ee_auto_calibrate", True)


        self.declare_parameter("ee_offset_right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_right_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_rpy_deg", [0.0, 0.0, 0.0])

        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.interface = str(self.get_parameter("interface").value)
        self.arm_velocity_limit = float(self.get_parameter("arm_velocity_limit").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame = str(self.get_parameter("ik_world_frame").value)
        self.ik_alpha = float(self.get_parameter("ik_alpha").value)
        self.ik_goal_filter_alpha = float(self.get_parameter("ik_goal_filter_alpha").value)
        self.ik_orientation_mode = str(self.get_parameter("ik_orientation_mode").value).lower()
        self.ik_max_ori_step_rad = float(self.get_parameter("ik_max_ori_step_rad").value)
        self.ee_auto_calibrate = bool(self.get_parameter("ee_auto_calibrate").value)

        self.declare_parameter("auto_reissue_goals", True)
        self.declare_parameter("goal_pos_tol", 0.01)
        self.declare_parameter("goal_ori_tol_deg", 3.0)

        self.auto_reissue_goals = bool(self.get_parameter("auto_reissue_goals").value)
        self.goal_pos_tol = float(self.get_parameter("goal_pos_tol").value)
        self.goal_ori_tol_deg = float(self.get_parameter("goal_ori_tol_deg").value)

        def _pvec(name):
            v = self.get_parameter(name).value
            return np.array(v, dtype=float)

        self._ee_off_right_xyz = _pvec("ee_offset_right_xyz")
        self._ee_off_right_rpy_deg = _pvec("ee_offset_right_rpy_deg")
        self._ee_off_left_xyz = _pvec("ee_offset_left_xyz")
        self._ee_off_left_rpy_deg = _pvec("ee_offset_left_rpy_deg")

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()
        self._last_q_target = np.zeros(14, dtype=float)
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02
        self._last_left_goal_raw = None
        self._last_right_goal_raw = None
        self._goal_left_filt = None
        self._goal_right_filt = None
        self._reset_after_home = False

        self._T_off_right_static = self._mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
        self._T_off_left_static = self._mk_static_T(self._ee_off_left_xyz, self._ee_off_left_rpy_deg)
        self._T_off_right_auto = None
        self._T_off_left_auto = None
        self._auto_done_right = False
        self._auto_done_left = False

        self.ik_solver = G1IKSolver(debug=False)
        if hasattr(self.ik_solver, "set_orientation_mode"):
            self.ik_solver.set_orientation_mode(self.ik_orientation_mode)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.home_right = np.array([0.90, -0.06, 0.04, -0.78, -0.07, -0.11, -0.30], dtype=float)
        self.home_left  = np.array([0.90, -0.06, 0.04, -0.78, -0.07, -0.11, -0.30], dtype=float)

        if not self.use_robot:
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/g1pilot/right_hand_goal", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/g1pilot/left_hand_goal", self._left_goal_callback, 10)
        self.create_subscription(Bool, "/g1pilot/arms_controlled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "/g1pilot/homming_arms", self._homming_callback, 10)

        if self.use_robot:
            self._init_robot_interface()

        self._last_tick_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop)

    def _mk_static_T(self, xyz, rpy_deg):
        rpy = np.radians(np.array(rpy_deg, dtype=float))
        R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
        return SE3(R, np.array(xyz, dtype=float))
    
    def _goal_error(self, side: str, T_goal: SE3):
        M_cur = self._fk_current_ee(side)
        if M_cur is None or T_goal is None:
            return None, None
        dp = float(np.linalg.norm(T_goal.translation - M_cur.translation))
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal.rotation)
        ang = 2.0 * math.atan2(
            math.sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z),
            abs(dq.w)
        )
        return dp, ang

    def _lowpass_goal(self, T_prev: SE3, T_new: SE3, alpha: float) -> SE3:
        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = _mat_to_quat_wxyz(T_prev.rotation)
        q1 = _mat_to_quat_wxyz(T_new.rotation)
        qf = (1 - alpha) * q0 + alpha * q1
        qf = qf / np.linalg.norm(qf)
        Rf = _quat_wxyz_to_matrix(qf)
        return SE3(Rf, p)

    def _limit_ori_step(self, R_cur: np.ndarray, R_des: np.ndarray, max_step: float) -> np.ndarray:
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        nrm = float(np.linalg.norm(aa))
        if nrm <= 1e-12 or nrm <= max_step:
            return R_des
        aa_lim = aa * (max_step / nrm)
        return R_cur @ pin.exp3(aa_lim)

    def _fk_current_ee(self, side: str):
        try:
            q_full = pin.neutral(self.ik_solver.model)
            cur_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()
            for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
                if ros_name in self.ik_solver._name_to_q_index:
                    q_full[self.ik_solver._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            fid = self.ik_solver._fid_right if side == 'right' else self.ik_solver._fid_left
            if fid is None:
                return None
            return self.ik_solver.data.oMf[fid]
        except Exception:
            return None

    def _gate_auto_calibration(self, T_goal_in: SE3, side: str):
        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2 * math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _apply_offsets_and_filters(self, side: str, T_goal_input: SE3):
        """Aplica offset estático + auto y filtra la meta; limita paso angular."""
        T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static
        T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
        auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

        if self.ee_auto_calibrate and not auto_done:
            M_cur_ok = self._gate_auto_calibration(T_goal_input, side)
            if M_cur_ok is not None:
                T_pre = T_goal_input * T_static
                T_auto_new = T_pre.inverse() * M_cur_ok
                if side == 'right':
                    self._T_off_right_auto = T_auto_new; self._auto_done_right = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated right: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                else:
                    self._T_off_left_auto = T_auto_new; self._auto_done_left = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated left: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                T_auto = T_auto_new

        T_raw = T_goal_input * T_static * (T_auto if T_auto is not None else SE3.Identity())

        if side == 'right':
            self._goal_right_filt = self._lowpass_goal(self._goal_right_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_right_filt
        else:
            self._goal_left_filt = self._lowpass_goal(self._goal_left_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_left_filt

        M_cur = self._fk_current_ee(side)
        if (M_cur is not None) and (T_use is not None):
            R_lim = self._limit_ori_step(M_cur.rotation, T_use.rotation, self.ik_max_ori_step_rad)
            T_use = SE3(R_lim, T_use.translation.copy())

        return T_use

    def _init_robot_interface(self):
        ChannelFactoryInitialize(0, self.interface)

        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', hg_LowState)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher('rt/arm_sdk', hg_LowCmd)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()

        self.kp_high = 300.0; self.kd_high = 3.0
        self.kp_low  = 150.0; self.kd_low  = 4.0
        self.kp_wrist= 40.0;  self.kd_wrist= 1.5

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)

    def _subscribe_motor_state(self):
        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def get_mode_machine(self):
        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_motor_q(self):
        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    def _assemble_full_from_last(self):
        full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[7 + i]
        return full

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

    def _arms_controlled_callback(self, msg: Bool):
        self.arms_enabled = msg.data
        if self.arms_enabled:
            try:
                cur = self.get_current_motor_q()
                left = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right= [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                pass
            self.get_logger().info("Arm ENABLED.")
        else:
            self.get_logger().info("Arm DISABLED")

    def _homming_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Moving both arms to HOME position.")
            self.homing_active = True
            self.homing_reached = False
            self._reset_after_home = False 
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()

    def _transform_pose_to_world(self, ps: PoseStamped) -> PoseStamped:
        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warning(f"[IK] TF {ps.header.frame_id}->{self.frame} failed: {e}")
            return ps

    def _right_goal_callback(self, msg: PoseStamped):
        if self.homing_active:
            return

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_right_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('right', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("right", T_goal_use)


    def _left_goal_callback(self, msg: PoseStamped):
        if self.homing_active:
            return

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_left_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('left', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("left", T_goal_use)


    def _compute_dt(self):
        now = time.time()
        if self._last_tick_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt

    def main_loop(self):
        # Actualiza buffer desde el robot (si aplica)
        if self.use_robot:
            robot_data = self.lowstate_subscriber.Read()
            if robot_data is not None:
                self.lowstate_buffer.SetData(robot_data)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = robot_data.motor_state[i].q
                    self.motor_state[i].dq = robot_data.motor_state[i].dq

        # Si brazos deshabilitados → mantener no-brazos y salir
        if not self.arms_enabled:
            self._hold_non_arm_joints()
            return

        # ===================== Generación de q_target =====================
        if self.homing_active:
            q_target = np.concatenate((self.home_left, self.home_right))
            if np.linalg.norm(q_target - self._last_q_target) < self.homing_tolerance:
                # HOME alcanzado
                self.homing_active = False
                self.homing_reached = True
                self._last_q_target = q_target.copy()

                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()

                self.ik_solver.set_current_configuration({
                    "left":  self.home_left.copy(),
                    "right": self.home_right.copy()
                })

                try:
                    q_full = pin.neutral(self.ik_solver.model)
                    for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_left[i]
                    for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_right[i]

                    pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                    pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)

                    T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
                    T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]

                    # Filtros y metas alineadas a HOME
                    self._goal_left_filt  = T_left.copy()
                    self._goal_right_filt = T_right.copy()
                    if hasattr(self.ik_solver, "set_goal"):
                        self.ik_solver.set_goal("left",  T_left.copy())
                        self.ik_solver.set_goal("right", T_right.copy())

                    # Marca que el próximo goal debe resetear historia
                    self._reset_after_home = True

                    self.get_logger().info("IK solver goals aligned with home pose.")
                except Exception as e:
                    self.get_logger().warning(f"Failed to align IK goals with home: {e}")

                self.get_logger().info("Home position reached.")

        elif self.homing_reached:
            q_target = np.concatenate((self.home_left, self.home_right))

        else:
            # Estado actual 29D (real o último target en sim)
            current_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()

            # Mantén semilla del IK con la postura actual para mejorar convergencia
            try:
                self.ik_solver.set_current_configuration({
                    "left":  self._last_q_target[0:7].copy(),
                    "right": self._last_q_target[7:14].copy()
                })
            except Exception:
                pass

            # ---------- Reinyecta SIEMPRE las metas latcheadas ----------
            if self._goal_left_filt is not None:
                self.ik_solver.set_goal("left", self._goal_left_filt)
            if self._goal_right_filt is not None:
                self.ik_solver.set_goal("right", self._goal_right_filt)

            # Pide targets al IK externo
            q_dict = self.ik_solver.get_joint_targets(current_all)
            q_target = np.zeros(14, dtype=float)
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

        # ===================== Suavizado y limitador de velocidad =====================
        dt = self._compute_dt()
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)

        q_unsmoothed = self._last_q_target + dq
        q_smooth = (1.0 - self.ik_alpha) * self._last_q_target + self.ik_alpha * q_unsmoothed
        self._last_q_target = q_smooth.copy()

        # ===================== Publicación =====================
        if self.use_robot:
            self.msg.mode_machine = self.get_mode_machine()
            self.msg.mode_pr = 1

            # Dummy (si el firmware lo requiere)
            try:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            except Exception:
                pass

            wrist_vals = {m.value for m in G1_29_JointWristIndex}
            for idx, jid in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[jid].mode = 1
                self.msg.motor_cmd[jid].q   = float(q_smooth[idx])
                self.msg.motor_cmd[jid].dq  = 0.0
                self.msg.motor_cmd[jid].tau = float(0.0)  # feedforward si quieres: self.tauff_target[idx]
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        else:
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

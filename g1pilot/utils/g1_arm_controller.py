#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import threading
import numpy as np
from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_pose
    _HAS_TF2 = True
except Exception:
    _HAS_TF2 = False

try:
    from geometry_msgs.msg import PoseStamped
except Exception:
    PoseStamped = None

from sensor_msgs.msg import JointState
import pinocchio as pin
from pinocchio import SE3
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
    JOINT_GROUPS,
    JOINT_NAMES_LEFT,
    JOINT_NAMES_RIGHT,
)

from g1pilot.utils.helpers import (
    clamp,
    wrap_to_pi,
    mat_to_quat_wxyz,
    quat_wxyz_to_matrix,
    quat_slerp,
    yaw_from_R,
)

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointIndex,
    DataBuffer,
)

from g1pilot.utils.arm_gui import ArmGUI, UiBridge

def _names_for(group):
    if group == "left":  return JOINT_NAMES_LEFT
    if group == "right": return JOINT_NAMES_RIGHT
    if group == "both":  return JOINT_NAMES_LEFT + JOINT_NAMES_RIGHT
    raise ValueError(f"Unknown group {group}")

os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")

JOINTID_TO_DUALINDEX = {}
for i, jid in enumerate([
    G1_29_JointArmIndex.kLeftShoulderPitch,
    G1_29_JointArmIndex.kLeftShoulderRoll,
    G1_29_JointArmIndex.kLeftShoulderYaw,
    G1_29_JointArmIndex.kLeftElbow,
    G1_29_JointArmIndex.kLeftWristRoll,
    G1_29_JointArmIndex.kLeftWristPitch,
    G1_29_JointArmIndex.kLeftWristyaw,
    G1_29_JointArmIndex.kRightShoulderPitch,
    G1_29_JointArmIndex.kRightShoulderRoll,
    G1_29_JointArmIndex.kRightShoulderYaw,
    G1_29_JointArmIndex.kRightElbow,
    G1_29_JointArmIndex.kRightWristRoll,
    G1_29_JointArmIndex.kRightWristPitch,
    G1_29_JointArmIndex.kRightWristYaw,
]):
    JOINTID_TO_DUALINDEX[jid.value] = i

class G1_29_ArmController:
    def __init__(self, ui_bridge: QtCore.QObject, controlled_arms: str = 'right',
                 show_ui: bool = True, ros_node: Node = None, ik_use_waist: bool = False,
                 ik_alpha: float = 0.2, ik_max_dq_step: float = 0.05, arm_velocity_limit: float = 2.0,
                 urdf_path: str = None, mesh_dir: str = None,):

        self.motor_state = [MotorState() for _ in range(35)]

        self._ros_node = ros_node
        self._joint_pub = None
        self.use_robot = True
        if self._ros_node is not None:
            if not self._ros_node.has_parameter('use_robot'):
                self._ros_node.declare_parameter('use_robot', True)
            self.use_robot = bool(self._ros_node.get_parameter('use_robot').value)
            qos = QoSProfile(depth=10)
            self._joint_pub = self._ros_node.create_publisher(JointState, "/joint_states", qos)

        self.topic_motion_cmd = 'rt/arm_sdk'
        self.topic_low_cmd    = 'rt/lowstate'

        self.q_target      = np.zeros(14)
        self.tauff_target  = np.zeros(14)
        self.kp_high = 300.0; self.kd_high = 3.0
        self.kp_low  = 150.0;  self.kd_low  = 4.0
        self.kp_wrist= 40.0;  self.kd_wrist= 1.5

        self.control_mode = False
        self.show_ui = bool(show_ui)

        self.all_motor_q = None
        self.arm_velocity_limit = float(arm_velocity_limit)
        self.control_dt = 1.0 / 250.0

        self._last_cmd_q = None
        self._last_tick_time = None
        self._speed_gradual_max = False

        self._bridge = ui_bridge
        self._gui = None

        self.controlled_arms = controlled_arms.lower().strip()
        if self.controlled_arms not in ("right", "left", "both"):
            self.controlled_arms = "right"

        self.gui_joint_ids   = JOINT_GROUPS[self.controlled_arms]
        self.gui_joint_names = _names_for(self.controlled_arms)
        self.gui_title       = {"right":"G1 – Right Arm Control","left":"G1 – Left Arm Control","both":"G1 – Both Arms Control"}[self.controlled_arms]

        self.sim_current_q_all = np.zeros(29, dtype=float)

        self.lowstate_buffer = DataBuffer()
        if self.use_robot:
            self.lowstate_subscriber = ChannelSubscriber(self.topic_low_cmd, hg_LowState)
            self.lowstate_subscriber.Init()
            self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
            self.subscribe_thread.start()

            self.lowcmd_publisher = ChannelPublisher(self.topic_motion_cmd, hg_LowCmd)
            self.lowcmd_publisher.Init()

            print("Waiting for first lowstate message...")
            while not self.lowstate_buffer.GetData():
                time.sleep(0.1)

            self.crc = CRC()
            self.msg = unitree_hg_msg_dds__LowCmd_()
            self.msg.mode_pr = 0
            self.msg.mode_machine = self.get_mode_machine()
            self.all_motor_q = self.get_current_motor_q()

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
        else:
            self.all_motor_q = self.sim_current_q_all.copy()

        self.ctrl_lock = threading.Lock()

        self._ik_enabled = True
        self._ik_alpha = float(ik_alpha)
        self._ik_max_dq_step = float(ik_max_dq_step)
        self._ik_damping = 1e-6
        self._ik_max_iter = 60
        self._ik_tol = 1e-4

        self._ik_use_waist = bool(ik_use_waist)
        self._ik_track_orientation = True
        self._ik_orientation_mode = "full"  # "full" | "yaw" | "none"
        self._ik_pos_gain = 1.0
        self._ik_ori_gain = 0.8
        self._ik_debug = False

        self._ik_adaptive_damping = True
        self._ik_sigma_min_thresh = 0.08
        self._ik_lambda_base = 1e-6
        self._ik_lambda_max  = 1e-1

        self._ik_max_ori_step_rad = 0.35  # ~20°
        self._goal_filter_alpha = 0.25  # [0..1]

        self._ik_have_joint_map = False
        self._ik_goal_left_raw  = None
        self._ik_goal_right_raw = None
        self._ik_goal_left      = None
        self._ik_goal_right     = None
        self._ik_q_prev_14  = None
        self._ik_q_prev_full = None
        self._log_ik_active = False

        self._ik_world_frame = 'pelvis'
        self._tf_buffer = None
        self._tf_listener = None

        self._ee_auto_calibrate = True
        self._ee_off_right_xyz = np.zeros(3)
        self._ee_off_right_rpy_deg = np.zeros(3)
        self._ee_off_left_xyz  = np.zeros(3)
        self._ee_off_left_rpy_deg  = np.zeros(3)

        self._T_off_right_static = None
        self._T_off_left_static  = None
        self._T_off_right_auto   = None
        self._T_off_left_auto    = None
        self._auto_done_right    = False
        self._auto_done_left     = False

        if self._ros_node is not None:
            def _safe_get(name, default):
                try:
                    if self._ros_node.has_parameter(name):
                        return self._ros_node.get_parameter(name).value
                except Exception:
                    pass
                return default

            self._ik_use_waist          = bool(_safe_get('ik_use_waist', self._ik_use_waist))
            self._ik_alpha              = float(_safe_get('ik_alpha', self._ik_alpha))
            self._ik_max_dq_step        = float(_safe_get('ik_max_dq_step', self._ik_max_dq_step))
            self.arm_velocity_limit     = float(_safe_get('arm_velocity_limit', self.arm_velocity_limit))
            self._ik_track_orientation  = bool(_safe_get('ik_track_orientation', self._ik_track_orientation))
            self._ik_orientation_mode   = str(_safe_get('ik_orientation_mode', self._ik_orientation_mode)).lower()
            self._ik_pos_gain           = float(_safe_get('ik_pos_gain', self._ik_pos_gain))
            self._ik_ori_gain           = float(_safe_get('ik_ori_gain', self._ik_ori_gain))
            self._ik_adaptive_damping   = bool(_safe_get('ik_adaptive_damping', self._ik_adaptive_damping))
            self._ik_sigma_min_thresh   = float(_safe_get('ik_sigma_min_thresh', self._ik_sigma_min_thresh))
            self._ik_max_ori_step_rad   = float(_safe_get('ik_max_ori_step_rad', self._ik_max_ori_step_rad))
            self._goal_filter_alpha     = float(_safe_get('ik_goal_filter_alpha', self._goal_filter_alpha))
            self._ik_world_frame        = str(_safe_get('ik_world_frame', self._ik_world_frame))
            self._ee_auto_calibrate     = bool(_safe_get('ee_auto_calibrate', self._ee_auto_calibrate))

            arr = _safe_get('ee_offset_right_xyz', list(self._ee_off_right_xyz))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_right_xyz = np.array(arr, dtype=float)
            arr = _safe_get('ee_offset_right_rpy_deg', list(self._ee_off_right_rpy_deg))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_right_rpy_deg = np.array(arr, dtype=float)

            arr = _safe_get('ee_offset_left_xyz', list(self._ee_off_left_xyz))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_left_xyz = np.array(arr, dtype=float)
            arr = _safe_get('ee_offset_left_rpy_deg', list(self._ee_off_left_rpy_deg))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_left_rpy_deg = np.array(arr, dtype=float)

            if _HAS_TF2:
                try:
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self._tf_buffer, self._ros_node)
                    print(f"[IK] TF listo. Frame objetivo: '{self._ik_world_frame}'", flush=True)
                except Exception as e:
                    print(f"[IK] No pude iniciar TF2: {e}", flush=True)

        try:
            self.pin = pin
            self.SE3 = SE3

            if urdf_path is None or mesh_dir is None:
                try:
                    pkg_share = get_package_share_directory('g1pilot')
                    urdf_path = urdf_path or os.path.join(pkg_share, 'description_files', 'urdf', '29dof.urdf')
                    mesh_dir  = mesh_dir  or os.path.join(pkg_share, 'description_files', 'meshes')
                except Exception:
                    self.get_logger().warning("Cannot find package 'g1pilot' for URDF path.")

            if urdf_path and os.path.exists(urdf_path):
                self.model, _, _ = self.pin.buildModelsFromUrdf(urdf_path, package_dirs=[mesh_dir] if mesh_dir else [])
                self.data = self.pin.Data(self.model)

                _joint_index_to_ros_name = JOINT_NAMES_ROS
                self._ros_joint_names = [_joint_index_to_ros_name[i] for i in range(29)]
                self._name_to_q_index = {}; self._name_to_v_index = {}
                for j in range(1, self.model.njoints):
                    jnt = self.model.joints[j]
                    if jnt.nq == 1:
                        nm = self.model.names[j]
                        if nm in self._ros_joint_names:
                            self._name_to_q_index[nm] = jnt.idx_q
                            self._name_to_v_index[nm] = jnt.idx_v

                self._frame_right = 'right_hand_point_contact'
                self._frame_left  = 'left_hand_point_contact'
                names_frames = [f.name for f in self.model.frames]
                self._fid_right = self.model.getFrameId(self._frame_right) if self._frame_right in names_frames else None
                self._fid_left  = self.model.getFrameId(self._frame_left ) if self._frame_left  in names_frames else None

                self._ros_to_g1_index = {v: k for k, v in _joint_index_to_ros_name.items()}

                def _mk_static_T(xyz, rpy_deg):
                    rpy = np.radians(np.array(rpy_deg, dtype=float))
                    R = self.pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
                    return self.SE3(R, np.array(xyz, dtype=float))
                self._T_off_right_static = _mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
                self._T_off_left_static  = _mk_static_T(self._ee_off_left_xyz,  self._ee_off_left_rpy_deg)

                self._ik_have_joint_map = True

                if self._ros_node is not None and PoseStamped is not None:
                    qos = rclpy.qos.QoSProfile(depth=10)
                    if self._fid_right is not None:
                        self._ros_node.create_subscription(
                            PoseStamped, '/g1pilot/right_hand_goal',
                            lambda msg: self._ik_target_cb('right', msg), qos)
                    if self._fid_left is not None:
                        self._ros_node.create_subscription(
                            PoseStamped, '/g1pilot/left_hand_goal',
                            lambda msg: self._ik_target_cb('left', msg), qos)
            else:
                print("[IK] URDF don't found", flush=True)
        except Exception as e:
            print(f"[IK] Failed initializing the IK: {e}", flush=True)
            self._ik_have_joint_map = False

        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()

    def move_arms_to_home(self):
        if not self.control_mode:
            return
        with self.ctrl_lock:
            if not hasattr(self, "q_target") or len(self.q_target) != 14:
                self.q_target = self.get_current_dual_arm_q().copy()
            if self.controlled_arms in ("right", "both"):
                right_home = np.array([20, 20, 20, 20, 20, 20, 20], dtype=float)
                self.q_target[7:14] = right_home
            if self.controlled_arms in ("left", "both"):
                left_home = np.array([20, 20, 20, 20, 20, 20, 20], dtype=float)
                self.q_target[0:7] = left_home
        self._ros_node.get_logger().info("[ArmGUI] Moving arms to home position")
        time_start = time.time()
        while time.time() - time_start < 5.0:
            with self.ctrl_lock:
                cur_q = self.get_current_dual_arm_q()
                if np.all(np.abs(cur_q - self.q_target) < 0.05):
                    break
            time.sleep(0.05)
        self._ros_node.get_logger().info("[ArmGUI] Move to home done")

    def _get_gui_initial_q(self):
        try:
            with self.ctrl_lock:
                base14 = self.q_target.copy() if hasattr(self, "q_target") and len(self.q_target) == 14 \
                         else self.get_current_dual_arm_q().copy()
        except Exception:
            base14 = self.get_current_dual_arm_q().copy()
        out = []
        for jid in self.gui_joint_ids:
            dual_idx = JOINTID_TO_DUALINDEX[jid]
            out.append(float(base14[dual_idx]))
        return out

    def _open_gui_if_needed(self):
        if not self.show_ui:
            return
        def _make_or_show():
            if self._gui is None:
                self._gui = ArmGUI(
                    title=self.gui_title,
                    joint_ids=self.gui_joint_ids,
                    joint_names=self.gui_joint_names,
                    get_initial_q_radians_callable=self._get_gui_initial_q
                )
                self._gui.valuesChanged.connect(self._on_gui_values)
            self._gui.move(100, 100)
            self._gui.show(); self._gui.raise_(); self._gui.activateWindow()
        self._bridge.runSignal.emit(_make_or_show)

    def _close_gui_if_needed(self):
        if self._gui is None:
            return
        self._bridge.runSignal.emit(self._gui.hide)

    def _on_gui_values(self, radians_list):
        if len(radians_list) != len(self.gui_joint_ids):
            return
        with self.ctrl_lock:
            if not hasattr(self, "q_target") or len(self.q_target) != 14:
                self.q_target = self.get_current_dual_arm_q().copy()
            for val, jid in zip(radians_list, self.gui_joint_ids):
                dual_idx = JOINTID_TO_DUALINDEX[jid]
                self.q_target[dual_idx] = float(val)

    def set_control_mode(self, enabled: bool):
        self.control_mode = enabled
        print(f"[ArmGUI] set_control_mode -> {enabled}", flush=True)
        if enabled:
            with self.ctrl_lock:
                cur14 = self.get_current_dual_arm_q().copy()
                self.q_target = cur14.copy()
                self.tauff_target = np.zeros_like(self.tauff_target)
                self._last_cmd_q = cur14.copy()
                self._ik_q_prev_14 = cur14.copy()
            if self.use_robot:
                self._arm_set_mode(mode=1)
            self._open_gui_if_needed()
        else:
            self._close_gui_if_needed()

    def get_mode_machine(self):
        if self.use_robot:
            msg = self.lowstate_buffer.GetData()
            return getattr(msg, "mode_machine", 0) if msg is not None else 0
        return 0

    def get_current_dual_arm_q(self):
        if self.use_robot:
            msg = self.lowstate_buffer.GetData()
            return np.array([msg.motor_state[id].q for id in G1_29_JointArmIndex], dtype=float)
        left = [self.sim_current_q_all[j] for j in LEFT_JOINT_INDICES_LIST]
        right= [self.sim_current_q_all[j] for j in RIGHT_JOINT_INDICES_LIST]
        return np.array(left + right, dtype=float)

    def get_current_motor_q(self):
        if self.use_robot:
            msg = self.lowstate_buffer.GetData()
            return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)
        return self.sim_current_q_all.copy()

    def _compute_dt(self):
        now = time.time()
        if self._last_tick_time is None:
            dt = self.control_dt
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt

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

    def _transform_pose_to_world(self, ps):
        if (self._ros_node is None) or (self._tf_buffer is None) or (not hasattr(ps, "header")):
            return ps
        src = ps.header.frame_id or ""
        target = self._ik_world_frame
        if not src or src == target:
            return ps
        try:
            tf = self._tf_buffer.lookup_transform(target, src, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self._ros_node.get_logger().warning(f"[IK] TF {src} -> {target} failed: {e}. Using pose.")
            return ps

    def _fk_current_ee(self, side):
        q = self.pin.neutral(self.model)
        cur_all = self.get_current_motor_q()
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        fid = self._fid_right if side == 'right' else self._fid_left
        if fid is None:
            return None
        return self.data.oMf[fid]

    def _gate_auto_calibration(self, T_goal_in, side):
        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2*math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _lowpass_goal(self, T_prev, T_new, alpha):
        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = mat_to_quat_wxyz(T_prev.rotation)
        q1 = mat_to_quat_wxyz(T_new.rotation)
        qf = quat_slerp(q0, q1, alpha)
        Rf = quat_wxyz_to_matrix(qf)
        return self.SE3(Rf, p)

    def _limit_ori_step(self, R_cur, R_des, max_step):
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        norm = float(np.linalg.norm(aa))
        if norm <= 1e-12 or norm <= max_step:
            return R_des
        aa_limited = aa * (max_step / norm)
        R_step = pin.exp3(aa_limited)
        return R_cur @ R_step

    def _ik_target_cb(self, side, msg):
        msg_tf = self._transform_pose_to_world(msg)
        if not self._ik_have_joint_map:
            return
        try:
            o = msg_tf.pose.orientation
            p = msg_tf.pose.position
            q = self.pin.Quaternion(o.w, o.x, o.y, o.z)
            T_goal_in = self.SE3(q.matrix(), np.array([p.x, p.y, p.z], dtype=float))

            T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static

            T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
            auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

            if self._ee_auto_calibrate and not auto_done:
                M_cur_ok = self._gate_auto_calibration(T_goal_in, side)
                if M_cur_ok is not None:
                    T_pre = T_goal_in * T_static
                    T_auto = T_pre.inverse() * M_cur_ok
                    if side == 'right':
                        self._T_off_right_auto = T_auto; self._auto_done_right = True
                    else:
                        self._T_off_left_auto  = T_auto; self._auto_done_left  = True
                    t = T_auto.translation
                    self._ros_node.get_logger().info(f"[IK] auto-calibrated {side}: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")

            T_raw = T_goal_in * T_static * (T_auto if T_auto is not None else self.SE3.Identity())

            if side == 'right':
                self._ik_goal_right_raw = T_raw
                self._ik_goal_right = self._lowpass_goal(self._ik_goal_right, T_raw, self._goal_filter_alpha)
            else:
                self._ik_goal_left_raw = T_raw
                self._ik_goal_left  = self._lowpass_goal(self._ik_goal_left, T_raw, self._goal_filter_alpha)

        except Exception as e:
            self._ros_node.get_logger().error(f"[IK] target_cb error: {e}")

    def _ordered_1d_names(self):
        out = []
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq == 1:
                nm = self.model.names[j]
                if nm in self._name_to_q_index:
                    out.append(nm)
        return out

    def _ik_solve_single(self, side, q_init=None):
        if not self._ik_have_joint_map:
            return None

        if side == 'right':
            fid = self._fid_right
            arm_ids = RIGHT_JOINT_INDICES_LIST
            arm_names = [
                "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
                "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint",
            ]
        else:
            fid = self._fid_left
            arm_ids = LEFT_JOINT_INDICES_LIST
            arm_names = [
                "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
                "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
            ]
        if fid is None:
            return None

        if q_init is not None and len(q_init) == self.model.nq:
            q = q_init.copy()
        elif hasattr(self, "_ik_q_prev_full") and (self._ik_q_prev_full is not None) and len(self._ik_q_prev_full) == self.model.nq:
            q = self._ik_q_prev_full.copy()
        else:
            q = self.pin.neutral(self.model)
            cur_all = self.get_current_motor_q()
            for jid_idx, ros_name in enumerate(self._ros_joint_names):
                if ros_name in self._name_to_q_index:
                    q[self._name_to_q_index[ros_name]] = float(cur_all[jid_idx])

        T_goal = self._ik_goal_right if side == 'right' else self._ik_goal_left
        if T_goal is None:
            return None

        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        M_cur0 = self.data.oMf[fid]
        R_limited = self._limit_ori_step(M_cur0.rotation, T_goal.rotation, self._ik_max_ori_step_rad)
        T_goal_limited = self.SE3(R_limited, T_goal.translation.copy())

        selected_names = []
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq != 1: continue
            nm = self.model.names[j]
            if nm in arm_names: selected_names.append(nm)
        if not selected_names:
            return None

        v_cols = [self._name_to_v_index[nm] for nm in selected_names if nm in self._name_to_v_index]
        q_idx  = [self._name_to_q_index[nm] for nm in selected_names if nm in self._name_to_q_index]
        if not v_cols or not q_idx or len(v_cols) != len(q_idx):
            return None

        for _ in range(self._ik_max_iter):
            self.pin.forwardKinematics(self.model, self.data, q)
            self.pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[fid]

            J6 = self.pin.computeFrameJacobian(self.model, self.data, q, fid, self.pin.LOCAL_WORLD_ALIGNED)
            J_eff = J6[:, v_cols]

            if self._ik_track_orientation and self._ik_orientation_mode != "none":
                if self._ik_orientation_mode == "yaw":
                    yaw_cur = yaw_from_R(M_cur.rotation)
                    yaw_des = yaw_from_R(T_goal_limited.rotation)
                    err_yaw = wrap_to_pi(yaw_des - yaw_cur)
                    pos_err = (T_goal_limited.translation - M_cur.translation).reshape(3)
                    err_use = np.hstack([pos_err * self._ik_pos_gain,
                                         [0.0, 0.0, err_yaw * self._ik_ori_gain]])
                    J_use = np.vstack([J_eff[:3, :],
                                       np.zeros((2, J_eff.shape[1])),
                                       J_eff[5:6, :]])
                    err_norm = float(np.linalg.norm(pos_err)) + abs(err_yaw)
                else:
                    err6 = self.pin.log(M_cur.inverse() * T_goal_limited).vector  # [dx,dy,dz, wx,wy,wz]
                    ori_norm = float(np.linalg.norm(err6[3:]))
                    if ori_norm > self._ik_max_ori_step_rad:
                        err6[3:] *= (self._ik_max_ori_step_rad / ori_norm)
                    J_use = np.vstack([
                        J_eff[:3, :] * self._ik_pos_gain,
                        J_eff[3:, :] * self._ik_ori_gain
                    ])
                    err_use = np.hstack([
                        err6[:3] * self._ik_pos_gain,
                        err6[3:] * self._ik_ori_gain
                    ])
                    err_norm = float(np.linalg.norm(err6))
            else:
                err_use = (T_goal_limited.translation - M_cur.translation).reshape(3) * self._ik_pos_gain
                J_use = J_eff[:3, :]
                err_norm = float(np.linalg.norm(err_use))

            if self._ik_debug:
                print(f"[IK] {side} ||err||={err_norm:.3e}", flush=True)
            if err_norm < self._ik_tol:
                break

            lam = self._ik_lambda_base
            if self._ik_adaptive_damping:
                try:
                    svals = np.linalg.svd(J_use, compute_uv=False)
                    sigma_min = float(np.min(svals)) if svals.size > 0 else 0.0
                except Exception:
                    sigma_min = 0.0
                if sigma_min < self._ik_sigma_min_thresh:
                    frac = clamp((self._ik_sigma_min_thresh - sigma_min) / max(1e-6, self._ik_sigma_min_thresh), 0.0, 1.0)
                    lam = self._ik_lambda_base + frac*(self._ik_lambda_max - self._ik_lambda_base)

            JJt = J_use @ J_use.T
            dq_red = J_use.T @ np.linalg.solve(JJt + lam * np.eye(J_use.shape[0]), err_use)

            dq_full = np.zeros(self.model.nv)
            for i, vi in enumerate(v_cols):
                step_i = float(np.clip(dq_red[i], -self._ik_max_dq_step, self._ik_max_dq_step))
                dq_full[vi] = step_i

            for i, (nm, qi) in enumerate(zip(selected_names, q_idx)):
                step = dq_full[v_cols[i]]
                q[qi] += step
                g1_idx = self._ros_to_g1_index.get(nm, None)
                if g1_idx is not None and g1_idx in JOINT_LIMITS_RAD:
                    lo, hi = JOINT_LIMITS_RAD[g1_idx]
                    q[qi] = float(np.clip(q[qi], lo, hi))

        self._ik_q_prev_full = q.copy()

        out7 = []
        for jid in arm_ids:
            ros_name = self._ros_joint_names[jid]
            if ros_name in self._name_to_q_index:
                val = float(q[self._name_to_q_index[ros_name]])
            else:
                dual_idx = JOINTID_TO_DUALINDEX[jid]
                val = float(self.get_current_dual_arm_q()[dual_idx])
            lo, hi = JOINT_LIMITS_RAD[jid]
            out7.append(float(np.clip(val, lo, hi)))
        return np.array(out7, dtype=float)

    def _ik_update_q_target(self, base_q14):
        base_q14 = np.asarray(base_q14, dtype=float).reshape(-1)
        if base_q14.size != 14:
            base_q14 = self.get_current_dual_arm_q().astype(float)

        left_q7 = right_q7 = None
        if self._ik_enabled:
            if self.controlled_arms in ('left','both') and self._ik_goal_left is not None:
                left_q7  = self._ik_solve_single('left')
            if self.controlled_arms in ('right','both') and self._ik_goal_right is not None:
                right_q7 = self._ik_solve_single('right')

        q_new = base_q14.copy()
        if left_q7  is not None: q_new[0:7]  = left_q7
        if right_q7 is not None: q_new[7:14] = right_q7

        if (self._ik_q_prev_14 is None) or (np.asarray(self._ik_q_prev_14).reshape(-1).size != 14):
            self._ik_q_prev_14 = q_new.copy()

        q_smooth = (1.0 - self._ik_alpha)*self._ik_q_prev_14 + self._ik_alpha*q_new
        self._ik_q_prev_14 = q_smooth.copy()
        return q_smooth

    def _ctrl_motor_state(self):
        all_joint_names = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
        while True:
            if self.control_mode:
                start = time.time()
                with self.ctrl_lock:
                    arm_q_target     = self.q_target.copy()
                    arm_tauff_target = self.tauff_target.copy()

                if self._ik_have_joint_map and self._ik_enabled and (self._ik_goal_left is not None or self._ik_goal_right is not None):
                    if not self._log_ik_active:
                        print("[IK] Activated: Using IK to update q_target", flush=True)
                        self._log_ik_active = True
                    arm_q_target = self._ik_update_q_target(arm_q_target)

                cliped = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)
                self._last_cmd_q = cliped.copy()

                if self.use_robot:
                    self.msg.mode_machine = self.get_mode_machine()
                    self.msg.mode_pr = 1

                    try:
                        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
                    except Exception:
                        pass

                    for idx, jid in enumerate(G1_29_JointArmIndex):
                        self.msg.motor_cmd[jid].mode = 1
                        self.msg.motor_cmd[jid].q   = float(cliped[idx])
                        self.msg.motor_cmd[jid].dq  = 0.0
                        self.msg.motor_cmd[jid].tau = float(arm_tauff_target[idx])
                        if jid.value in {m.value for m in G1_29_JointWristIndex}:
                            self.msg.motor_cmd[jid].kp = self.kp_wrist
                            self.msg.motor_cmd[jid].kd = self.kd_wrist
                        else:
                            self.msg.motor_cmd[jid].kp = self.kp_low
                            self.msg.motor_cmd[jid].kd = self.kd_low

                    self.msg.crc = self.crc.Crc(self.msg)
                    self.lowcmd_publisher.Write(self.msg)
                else:
                    for idx, jid in enumerate(LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST):
                        self.sim_current_q_all[jid] = float(cliped[idx])
                    if self._joint_pub is not None:
                        js = JointState()
                        js.header.stamp = self._ros_node.get_clock().now().to_msg()
                        js.name = all_joint_names
                        js.position = [self.sim_current_q_all[i] for i in range(29)]
                        self._joint_pub.publish(js)

                if self._gui is not None and self.control_mode:
                    if self.use_robot:
                        current_q = self.get_current_dual_arm_q()
                    else:
                        left = [self.sim_current_q_all[j] for j in LEFT_JOINT_INDICES_LIST]
                        right = [self.sim_current_q_all[j] for j in RIGHT_JOINT_INDICES_LIST]
                        current_q = np.array(left + right, dtype=float)
                    gui_vals = []
                    for jid in self.gui_joint_ids:
                        dual_idx = JOINTID_TO_DUALINDEX[jid]
                        gui_vals.append(float(current_q[dual_idx]))
                    self._bridge.runSignal.emit(lambda: self._gui.set_slider_values(gui_vals))

                if self._speed_gradual_max:
                    t_elapsed = time.time() - start
                    self.arm_velocity_limit = 20.0 + 10.0 * min(1.0, t_elapsed / 5.0)

                dt = time.time() - start
                time.sleep(max(0.0, self.control_dt - dt))
            else:
                if self.use_robot:
                    self._hold_non_arm_joints()
                time.sleep(self.control_dt)

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)


def main():
    rclpy.init()
    node = Node("g1_arm_controller")
    if not node.has_parameter('use_robot'):
        node.declare_parameter('use_robot', True)

    defaults = {
        'ik_orientation_mode': 'yaw',
        'ik_goal_filter_alpha': 0.3,
        'ik_sigma_min_thresh': 0.08,
        'ik_max_ori_step_rad': 0.35,
        'ik_adaptive_damping': True,
        'ik_pos_gain': 1.0,
        'ik_ori_gain': 0.7,
        'ik_use_waist': True,
        'ik_alpha': 0.2,
        'ik_max_dq_step': 0.05,
        'arm_velocity_limit': 2.0,
        'ik_world_frame': 'pelvis',
        'ee_auto_calibrate': True,
    }
    for k, v in defaults.items():
        try:
            if not node.has_parameter(k):
                node.declare_parameter(k, v)
        except Exception:
            pass

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication([])
    bridge = UiBridge()

    ctrl = G1_29_ArmController(
        bridge,
        controlled_arms='right',
        show_ui=True,
        ros_node=node,
        ik_use_waist=True,
        ik_alpha=0.2,
        ik_max_dq_step=0.05,
        arm_velocity_limit=2.0,
    )
    ctrl.set_control_mode(True)

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

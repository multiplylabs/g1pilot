import math
import numpy as np
from PyQt6 import QtCore, QtWidgets

from g1pilot.utils.joints_names import (
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
    JOINT_NAMES_LEFT,
    JOINT_NAMES_RIGHT,
)

from g1pilot.utils.helpers import clamp_joint_vector

class ArmGUI(QtWidgets.QWidget):
    valuesChanged = QtCore.pyqtSignal(object)

    def __init__(self, title, joint_ids, joint_names, get_initial_q_radians_callable, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setMinimumWidth(720 if len(joint_ids) == 14 else 520)
        self.joint_ids   = joint_ids[:]
        self.joint_names = joint_names[:]

        root = QtWidgets.QVBoxLayout(self)
        self.setLayout(root)
        columns = QtWidgets.QHBoxLayout()
        root.addLayout(columns, stretch=1)
        self.sliders = []; self.value_labels = []

        init_q = get_initial_q_radians_callable() or [0.0]*len(self.joint_ids)
        if len(init_q) != len(self.joint_ids):
            init_q = [0.0]*len(self.joint_ids)
        init_q = clamp_joint_vector(init_q, self.joint_ids)

        def make_slider_row(name, jidx, deg0):
            row = QtWidgets.QHBoxLayout()
            lab = QtWidgets.QLabel(name); lab.setFixedWidth(180)
            lo_rad, hi_rad = JOINT_LIMITS_RAD[jidx]
            lo_deg = int(round(math.degrees(lo_rad))); hi_deg = int(round(math.degrees(hi_rad)))
            sld = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            sld.setMinimum(lo_deg); sld.setMaximum(hi_deg)
            sld.setSingleStep(1); sld.setPageStep(5)
            sld.setTickInterval(max(5, (hi_deg - lo_deg) // 12))
            sld.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
            deg0 = max(lo_deg, min(hi_deg, deg0)); sld.setValue(deg0)
            val_lab = QtWidgets.QLabel(f"{deg0:>4}°"); val_lab.setFixedWidth(60)
            return row, lab, sld, val_lab

        if len(self.joint_ids) == 14:
            left_ids  = LEFT_JOINT_INDICES_LIST; right_ids = RIGHT_JOINT_INDICES_LIST
            left_names  = JOINT_NAMES_LEFT;      right_names = JOINT_NAMES_RIGHT
            col_left  = QtWidgets.QVBoxLayout(); col_right = QtWidgets.QVBoxLayout()
            columns.addLayout(col_left, 1); columns.addSpacing(16); columns.addLayout(col_right, 1)
            left_title  = QtWidgets.QLabel("Left Arm");  right_title = QtWidgets.QLabel("Right Arm")
            left_title.setStyleSheet("font-weight:600;"); right_title.setStyleSheet("font-weight:600;")
            col_left.addWidget(left_title); col_right.addWidget(right_title)

            for i, (name, jidx) in enumerate(zip(left_names, left_ids)):
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                col_left.addLayout(row); self.sliders.append(sld); self.value_labels.append(val_lab)
            offset = 7
            for k, (name, jidx) in enumerate(zip(right_names, right_ids)):
                i = offset + k
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                col_right.addLayout(row); self.sliders.append(sld); self.value_labels.append(val_lab)
        else:
            single_col = QtWidgets.QVBoxLayout(); columns.addLayout(single_col, 1)
            for i, (name, jidx) in enumerate(zip(self.joint_names, self.joint_ids)):
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                single_col.addLayout(row); self.sliders.append(sld); self.value_labels.append(val_lab)

        btns = QtWidgets.QHBoxLayout()
        btn_center = QtWidgets.QPushButton("Center (0°)")
        btn_center.clicked.connect(self._center_all)
        btns.addStretch(1); btns.addWidget(btn_center); root.addLayout(btns)

    def _on_slider(self, idx, value_deg, val_label):
        val_label.setText(f"{int(value_deg):>4}°")
        vals_rad = []
        for sld, jidx in zip(self.sliders, self.joint_ids):
            lo_rad, hi_rad = JOINT_LIMITS_RAD[jidx]
            rad = math.radians(sld.value()); vals_rad.append(float(np.clip(rad, lo_rad, hi_rad)))
        self.valuesChanged.emit(vals_rad)

    def _center_all(self):
        for sld, jidx in zip(self.sliders, self.joint_ids):
            lo_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][0])))
            hi_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][1])))
            center = 0 if lo_deg <= 0 <= hi_deg else int((lo_deg + hi_deg) / 2)
            sld.setValue(center)

    def update_from_robot_pose(self, q_rad_in_gui_order):
        if not q_rad_in_gui_order or len(q_rad_in_gui_order) != len(self.joint_ids):
            return
        q_rad_in_gui_order = clamp_joint_vector(q_rad_in_gui_order, self.joint_ids)
        for i, (val, jidx) in enumerate(zip(q_rad_in_gui_order, self.joint_ids)):
            lo_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][0])))
            hi_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][1])))
            deg = int(round(math.degrees(val))); deg = max(lo_deg, min(hi_deg, deg))
            self.sliders[i].blockSignals(True); self.sliders[i].setValue(deg)
            self.value_labels[i].setText(f"{deg:>4}°"); self.sliders[i].blockSignals(False)

class UiBridge(QtCore.QObject):
    runSignal = QtCore.pyqtSignal(object)
    def __init__(self):
        super().__init__()
        self.runSignal.connect(self._run)
    @QtCore.pyqtSlot(object)
    def _run(self, fn):
        try:
            fn()
        except Exception as e:
            print(f"[UiBridge] Error callable: {e}", flush=True)
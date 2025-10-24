#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QLabel, QVBoxLayout
)
from PyQt6.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from g1pilot.utils.window_style import DarkStyle


class StreamDeck(Node):
    def __init__(self):
        super().__init__('stream_deck')

        self.pub_start = self.create_publisher(Bool, '/g1pilot/start', 10)
        self.pub_start_balancing = self.create_publisher(Bool, '/g1pilot/start_balancing', 10)
        self.pub_arms_enabled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 10)
        self.pub_arms_home = self.create_publisher(Bool, '/g1pilot/arms/home', 10)
        self.pub_left_hand = self.create_publisher(String, '/g1pilot/dx3/hand_action/left', 10)
        self.pub_right_hand = self.create_publisher(String, '/g1pilot/dx3/hand_action/right', 10)
        self.pub_emergency_stop = self.create_publisher(Bool, '/g1pilot/emergency_stop', 10)

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_str(self, pub, text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)

class ButtonGUI(QWidget):

    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.button_states = {}
        self.hand_pairs = {
            "left": {"open": (2, 0), "close": (2, 1)},
            "right": {"open": (2, 2), "close": (2, 3)},
        }

        self.setWindowTitle("DIGITAL STREAMDECK")
        self.init_ui()
        self.apply_style()
        

    def init_ui(self):
        main_layout = QVBoxLayout()

        grid = QGridLayout()
        grid.setSpacing(10)
        rows, cols = 5, 5
        self.buttons = {}

        button_actions = {
            (0, 0): ("START", lambda: self.toggle_button((0, 0), self.node.pub_start)),
            (0, 1): ("START\nBALANCING", lambda: self.toggle_button((0, 1), self.node.pub_start_balancing)),
            (1, 0): ("ENABLE\nMANIPULATION", lambda: self.toggle_button((1, 0), self.node.pub_arms_enabled)),
            (1, 1): ("HOMING\nARMS", lambda: self.toggle_button((1, 1), self.node.pub_arms_home)),

            (2, 0): ("OPEN\nLEFT\nHAND", lambda: self.toggle_hand("left", "open", self.node.pub_left_hand)),
            (2, 1): ("CLOSE\nLEFT\nHAND", lambda: self.toggle_hand("left", "close", self.node.pub_left_hand)),
            (2, 2): ("OPEN\nRIGHT\nHAND", lambda: self.toggle_hand("right", "open", self.node.pub_right_hand)),
            (2, 3): ("CLOSE\nRIGHT\nHAND", lambda: self.toggle_hand("right", "close", self.node.pub_right_hand)),

            (4, 4): ("EMERGENCY\nSTOP", lambda: self.emergency_stop()),
        }

        for r in range(rows):
            for c in range(cols):
                btn = QPushButton()
                btn.setMinimumSize(120, 80)

                action = button_actions.get((r, c))
                if action is None:
                    btn.setEnabled(False)
                    btn.setFlat(True)
                    btn.setStyleSheet("""
                        QPushButton {
                            background-color: #1e1e1e;
                            border: 1px solid #333;
                            border-radius: 10px;
                        }
                    """)
                else:
                    label, func = action
                    btn.setText(label)
                    btn.clicked.connect(func)
                    if (r, c) == (4, 4):
                        btn.setStyleSheet("""
                            QPushButton {
                                background-color: #b00000;
                                color: white;
                                font-weight: bold;
                                border: 1px solid #ff4444;
                                border-radius: 10px;
                            }
                            QPushButton:hover {
                                background-color: #ff0000;
                                border: 1px solid #ff6666;
                            }
                        """)

                grid.addWidget(btn, r, c)
                self.buttons[(r, c)] = btn
                self.button_states[(r, c)] = False

        main_layout.addLayout(grid)
        self.setLayout(main_layout)

    def apply_style(self):
        self.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #ffffff;
                font-size: 15px;
                font-weight: 600;
                border: 1px solid #444;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover:enabled {
                background-color: #3c3c3c;
                border: 1px solid #66b3ff;
            }
            QPushButton:pressed {
                background-color: #1f5fa1;
                border: 1px solid #80c4ff;
            }
            QPushButton:disabled {
                color: #555;
                background-color: #1e1e1e;
                border: 1px solid #2a2a2a;
            }
            QWidget {
                background-color: #111;
            }
        """)

    def set_button_active(self, pos, active=True):
        btn = self.buttons[pos]
        if active:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #80ff80;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #5fdc5f;
                }
            """)
        else:
            btn.setStyleSheet("")
            self.apply_style()

        self.button_states[pos] = active

    def toggle_button(self, pos, pub):
        new_state = not self.button_states[pos]
        self.set_button_active(pos, new_state)
        self.node.publish_bool(pub, new_state)

    def toggle_hand(self, hand_side, action, pub):
        hand_pair = self.hand_pairs[hand_side]
        this_pos = hand_pair[action]
        other_pos = hand_pair["close" if action == "open" else "open"]

        self.set_button_active(this_pos, True)
        self.set_button_active(other_pos, False)

        self.node.publish_str(pub, action)

    def emergency_stop(self):
        self.node.publish_bool(self.node.pub_emergency_stop, True)
        btn = self.buttons[(4, 4)]
        btn.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
                color: white;
                font-weight: bold;
                border: 2px solid #ff6666;
                border-radius: 10px;
            }
        """)


def main():
    rclpy.init()
    node = StreamDeck()

    app = QApplication(sys.argv)
    DarkStyle(app)
    gui = ButtonGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    app.quit()


if __name__ == '__main__':
    main()

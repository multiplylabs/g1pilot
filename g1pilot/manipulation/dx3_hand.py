#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

CLOSE_RIGHT_VALUES = [-0.05, -0.99, -1.68, 1.56, 1.70, 1.56, 1.70]
CLOSE_LEFT_VALUES  = [-0.02,  0.47,  1.89, -1.44, -1.89, -1.52, -1.92]
OPEN_VALUES        = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class DX3Controller(Node):
    def __init__(self):
        super().__init__('dx3_hand_controller')
        self.declare_parameter("interface", "eth0")
        self.declare_parameter("arm_controlled", "both")
        interface = self.get_parameter("interface").get_parameter_value().string_value
        arm_controlled = self.get_parameter("arm_controlled").get_parameter_value().string_value

        self.right_action = None
        self.left_action = None
        self.right_target = OPEN_VALUES
        self.left_target = OPEN_VALUES
        self.total_motors = 7

        ChannelFactoryInitialize(0, interface)

        if arm_controlled in ["right", "both"]:
            self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
            self.right_pub.Init()
            self.right_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
            self.right_sub.Init(self.right_callback)
            self.create_subscription(String, "g1pilot/dx3/right/hand_action", self.right_action_callback, 10)

        if arm_controlled in ["left", "both"]:
            self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
            self.left_pub.Init()
            self.left_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.left_sub.Init(self.left_callback)
            self.create_subscription(String, "g1pilot/dx3/left/hand_action", self.left_action_callback, 10)

        self.create_timer(0.05, self.publish_commands)

    def right_action_callback(self, msg: String):
        if msg.data not in ["open", "close"]:
            return
        if msg.data != self.right_action:
            self.right_action = msg.data
            self.right_target = CLOSE_RIGHT_VALUES if msg.data == "close" else OPEN_VALUES

    def left_action_callback(self, msg: String):
        if msg.data not in ["open", "close"]:
            return
        if msg.data != self.left_action:
            self.left_action = msg.data
            self.left_target = CLOSE_LEFT_VALUES if msg.data == "close" else OPEN_VALUES

    def left_callback(self, msg: HandState_):
        self.get_logger().debug(f'Left hand state received: {msg}')

    def right_callback(self, msg: HandState_):
        self.get_logger().debug(f'Right hand state received: {msg}')

    def create_cmd(self, values):
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(self.total_motors):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q = values[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 1.5
            cmd.motor_cmd[i].kd = 0.2
        return cmd

    def publish_commands(self):
        if hasattr(self, "right_pub") and self.right_action is not None:
            self.right_pub.Write(self.create_cmd(self.right_target))
        if hasattr(self, "left_pub") and self.left_action is not None:
            self.left_pub.Write(self.create_cmd(self.left_target))

def main(args=None):
    rclpy.init(args=args)
    node = DX3Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

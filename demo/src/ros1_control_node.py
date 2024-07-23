#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from tm_msgs.srv import SetPositions, SetPositionsRequest, SetIO, SetIORequest
from tm_msgs.msg import FeedbackState
from rospy import Timer
import time
import numpy as np


class MyROSNode:
    def __init__(self):
        rospy.init_node("my_ros_node")

        self.state_sub = rospy.Subscriber("/feedback_states", FeedbackState, self.state_callback)
        self.target_point_set_sub = rospy.Subscriber("/target_point_set", Float64MultiArray, self.target_point_set_callback)
        self.timer = Timer(rospy.Duration(0.1), self.check_callback)

        self.custom_service_client = rospy.ServiceProxy("tm_driver/set_positions", SetPositions)
        self.gripper_service_client = rospy.ServiceProxy("tm_driver/set_io", SetIO)

        self.flag = 0
        self.cart_pos_state = [10.0] * 6
        self.target_set = None

    def target_point_set_callback(self, msg):
        self.flag = 1
        self.target_set = np.array(msg.data)

        for i, pose in enumerate(self.target_set):
            if len(pose) == 1:
                binary = pose
                rospy.loginfo(f"gripper {i} open={binary}")
            elif len(pose) == 6:
                x, y, z, roll, pitch, yaw = pose
                rospy.loginfo(f"Pose {i}: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
            else:
                rospy.logerr(f"Invalid pose {i}: {pose}")

    def state_callback(self, msg):
        for i in range(6):
            self.joint_pos_state[i] = msg.joint_pos[i]
            self.cart_pos_state[i] = msg.tool_pose[i]

    def check_callback(self, event):
        if self.flag == 1:
            if len(self.target_set):
                if len(self.target_set[0]) > 1:
                    self.target_point = self.target_set[0]
                    self.send_position_cmd(self.target_set.pop(0))
                    self.flag = 2
                else:
                    tmp = self.target_set.pop(0)
                    print(f"tmp: {tmp}")
                    self.gripper_controller(tmp)
            else:
                self.flag = 0

        if self.flag == 2:
            current_pos = self.cart_pos_state
            print(f"current_pos: {current_pos}, self.target_point: {self.target_point}")
            arrived = all(abs(current_pos[i] - self.target_point[i]) < 0.0001 for i in range(6))
            print(f"arrived: {arrived}")
            if arrived:
                self.flag = 1

    def gripper_controller(self, input):
        srv = SetIORequest()
        srv.module = 1
        srv.type = 1
        srv.pin = 0
        srv.state = 1.0 if input[0] == 1. else 0.0
        time.sleep(2)
        self.gripper_service_client(srv)
        time.sleep(2)

    def send_position_cmd(self, position_cmd):
        print(f"position_cmd: {position_cmd}")
        position_cmd = np.array(position_cmd)
        srv = SetPositionsRequest()
        srv.motion_type = 2
        srv.positions = position_cmd
        srv.velocity = 2.0  # rad/s
        srv.acc_time = 0.8
        srv.blend_percentage = 10
        srv.fine_goal = False

        try:
            response = self.custom_service_client(srv)
            if response.ok:
                rospy.loginfo("SetPositions to robot")
            else:
                rospy.logwarn("SetPositions to robot, but response not yet ok")
        except rospy.ServiceException as e:
            rospy.logerr("Error SetPositions to robot: %s", str(e))

        rospy.loginfo("Start to go to point")

if __name__ == '__main__':
    try:
        node = MyROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

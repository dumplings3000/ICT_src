#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
from tm_msgs.srv import SetPositions, SetPositionsRequest, SetIO, SetIORequest
from tm_msgs.msg import FeedbackState
from rospy import Timer
import time
import numpy as np
import os


class MyROSNode:
    def __init__(self):
        rospy.init_node("my_ros_node")

        self.state_sub = rospy.Subscriber("/feedback_states", FeedbackState, self.state_callback)
        self.start_sub = rospy.Subscriber("/start_cmd", Int32, self.start_callback)
        self.timer = Timer(rospy.Duration(0.1), self.check_callback)

        self.custom_service_client = rospy.ServiceProxy("tm_driver/set_positions", SetPositions)
        self.gripper_service_client = rospy.ServiceProxy("tm_driver/set_io", SetIO)

        self.flag = 0
        self.mode = None  # planning in joint space or cartesian
        self.joint_pos_state = [10.0] * 6
        self.cart_pos_state = [10.0] * 6
        self.joint_point_set = None
        self.cart_point_set = None

        # Get the current script's file path
        self.current_file_path = os.path.dirname(os.path.abspath(__file__))
        # joint part
        self.joint_point_set = self.load_txt_file(self.current_file_path + "/" + 'joint_points.txt')
        # cartesian part
        self.cart_point_set = self.load_txt_file(self.current_file_path + "/" + 'cart_points.txt')

    def gripper_controller(self, input):
        srv = SetIORequest()
        srv.module = 1
        srv.type = 1
        srv.pin = 0
        srv.state = 1.0 if input[0] == 1. else 0.0
        time.sleep(2)
        self.gripper_service_client(srv)
        time.sleep(2)

    def state_callback(self, msg):
        for i in range(6):
            self.joint_pos_state[i] = msg.joint_pos[i]
            self.cart_pos_state[i] = msg.tool_pose[i]

    def start_callback(self, msg):
        if msg.data == 1:
            print(f"using joint")
            self.flag = 1
            self.mode = "joint"
            self.points_set = self.joint_point_set.copy()
        elif msg.data == 2:
            print(f"using cartesian")
            self.flag = 1
            self.mode = "cartesian"
            self.points_set = self.cart_point_set.copy()
        else:
            if msg.data >= 3 and msg.data <= 17:
                self.flag = 1
                self.mode = "cartesian"
                self.cart_point_set = self.load_txt_file(self.current_file_path + "/point" + str(msg.data-3) + ".txt")
                print(self.cart_point_set)
                self.points_set = self.cart_point_set.copy()
            else:
                print(f"input 1 for joint, 2 for cartesian")

    def check_callback(self, event):
        if self.flag == 1:
            if len(self.points_set):
                if len(self.points_set[0]) > 1:
                    self.target_point = self.points_set[0]
                    self.send_position_cmd(self.points_set.pop(0))
                    self.flag = 2
                else:
                    tmp = self.points_set.pop(0)
                    print(f"tmp: {tmp}")
                    self.gripper_controller(tmp)
            else:
                self.flag = 0

        if self.flag == 2:
            current_pos = self.joint_pos_state if self.mode == "joint" else self.cart_pos_state
            print(f"current_pos: {current_pos}, self.target_point: {self.target_point}")
            arrived = all(abs(current_pos[i] - self.target_point[i]) < 0.0001 for i in range(6))
            print(f"arrived: {arrived}")
            if arrived:
                self.flag = 1

    def send_position_cmd(self, position_cmd):
        print(f"position_cmd: {position_cmd}")
        position_cmd = np.array(position_cmd)
        srv = SetPositionsRequest()
        srv.motion_type = 1 if self.mode == "joint" else 2
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

    def load_txt_file(self, filename):
        data_list = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    # Remove trailing whitespace and newline characters
                    line = line.strip()
                    # Split the line into a list of values using ',' as the delimiter
                    values = line.split(',')
                    values = [float(x) for x in values]
                    # Append the list of values to the 2D data_list
                    data_list.append(values)
            return data_list
        except FileNotFoundError:
            print(f"The file '{filename}' was not found.")
            return None
        except Exception as e:
            print(f"An error occurred: {str(e)}")
            return None


if __name__ == '__main__':
    try:
        node = MyROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

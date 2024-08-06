#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
from tm_msgs.srv import SetPositions, SetPositionsRequest, SetIO, SetIORequest
from tm_msgs.msg import FeedbackState
from rospy import Timer
import time
import numpy as np
import os
from demo.srv import set_position, set_positionResponse

class MyROSNode:
    def __init__(self):
        rospy.init_node("my_ros_node")

        # ... (rest of the initialization code)

        self.move_to_goal_service = rospy.Service("/move_to_goal", set_position, self.move_to_goal)
        self.state_sub = rospy.Subscriber("/feedback_states", FeedbackState, self.state_callback)
        self.position_service_client = rospy.ServiceProxy("tm_driver/set_positions", SetPositions)
        self.gripper_service_client = rospy.ServiceProxy("tm_driver/set_io", SetIO)
        # self.timer = Timer(rospy.Duration(1), self.check_callback)
        self.joint_pos_state = [10.0] * 6
        self.cart_pos_state = [10.0] * 6
        self.goal = [10.0] * 6
        self.mode = 1  # 1 for joint, 2 for cartesian
        self.arrived = False
        self.flag = True

    def state_callback(self, msg):
        # print(f"self.joint_pos_state: {self.joint_pos_state}")
        # print(f"self.cart_pos_state: {self.cart_pos_state}")
        for i in range(6):
            self.joint_pos_state[i] = msg.joint_pos[i]
            self.cart_pos_state[i] = msg.tool_pose[i]


    def check_callback(self):
        # print(f"update arrived value: {self.arrived}")
        if len(self.goal) == 1:
            self.arrived = True
            # print(f"arrived value: {self.arrived}, goal: {len(self.goal)}")
        elif len(self.goal) == 6:
            if self.mode == 1:
                self.arrived = all(abs(self.joint_pos_state[i] - self.goal[i]) < 0.0001 for i in range(6))
                # print(f"arrived value: {self.arrived}, goal: {len(self.goal)}")
            elif self.mode == 2:
                self.arrived = all(abs(self.cart_pos_state[i] - self.goal[i]) < 0.0001 for i in range(6))
                # print(f"arrived value: {self.arrived}, goal: {len(self.goal)}")

        else:
            print(f"Invalid current_state or goal_pose")

    def move_to_goal(self, request):
        self.mode = request.type
        self.goal = request.value
        # print("update goal value: ", self.goal)
        if len(self.goal) == 6:
            position_cmd = np.array(self.goal)
            srv = SetPositionsRequest()
            srv.motion_type =  self.mode 
            srv.positions = position_cmd
            srv.velocity = 1.5  # rad/s
            srv.acc_time = 0.8
            srv.blend_percentage = 10
            srv.fine_goal = False
            self.position_service_client(srv)
            self.check_callback()
            while not self.arrived:
                # if self.mode == 1:
                #     print(f"mode1 current_pos: {self.joint_pos_state}, self.target_point: {self.goal}")
                # elif self.mode == 2:
                #     print(f"mode2 current_pos: {self.cart_pos_state}, self.target_point: {self.goal}")
                # print(f"arrived: {self.arrived}")
                self.check_callback()
            
            self.goal = [10.0] * 6
            success = set_positionResponse()
            success.response = True
            self.flag = True
            return success
        
        elif len(self.goal) == 1:
            srv = SetIORequest()
            srv.module = 1
            srv.type = 1
            srv.pin = 0
            srv.state = 1.0 if self.goal[0] == 1. else 0.0
            time.sleep(2)
            self.gripper_service_client(srv)
            time.sleep(1)
            self.flag = True
            return True
        else:
            print(f"Unvalid input")
            self.flag = True
            return False
            

if __name__ == '__main__':
    try:
        node = MyROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import random
import rospy
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np
from demo.srv import set_position, set_positionRequest
import copy


class Node1:
    def __init__(self):
        rospy.init_node('node_1', anonymous = True)

        # Create a CV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.move_to_goal_client = rospy.ServiceProxy("/move_to_goal", set_position)
        rospy.Subscriber('/start_the_game', Int32, self.stage_switch)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_light_blue = np.array([90, 50, 50])
        self.upper_light_blue = np.array([130, 255, 255])
        self.state = None # The initiate state is set to None to oprevent the arm is in the place that isn't able to see the board
        self.board = np.zeros((3,3))
        self.init_observe_pose_joint = [1.5423415158118359, 0.21664547487209915, 1.1669678949007314, 0.18664356703278423, 1.571445871784767, -0.028342209203220715]
        self.init_observe_pose = [0.14025527954101563, 0.6993024291992188, 0.525866455078125, 3.1399952895679637, -0.0, 3.140006208528433]
        self.init_grasp_pose = [-0.140, 0.55, 0.23, 3.14, 0., 3.14]
        self.middle_place_pose = [0.140, 0.75, 0.21, 3.14, 0., 3.14]
        self.block_num = 0 # Start with 0, end with up to 5 since the arm has 5 blocks to grasp
        # self.last_board = np.zeros((3,3))

    def image_callback(self, msg):
        try:
            # while self.board != self.last_board:
            #     break
            self.board = np.zeros((3,3))
            # Get the blocks of re and blue respectively
            red_blocks = self.get_red_blocks(msg=msg)
            blue_blocks = self.get_blue_blocks(msg=msg)

            # Update them respectively
            self.update_board(red_blocks, self.board)
            self.update_board(blue_blocks, self.board)


            # self.winner =  self.check_winner(self.board)
            
            # if self.winner != None:
            #     winner_name = "red" if self.winner== 1 else "blue"
            #     print(f"winner is {winner_name}")
            #     self.state = "finish"


        except Exception as e:
            rospy.logerr("Error processing image: {}".format(str(e)))
        


    def get_red_blocks(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")    
        # Convert BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        hsv_img = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
        
        mask_red = cv2.inRange(hsv_img, self.lower_red1, self.upper_red1)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_blocks = []
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 100:
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    blue_blocks.append((cx, cy, 1)) # 1 for red, 2 for blue
        return blue_blocks

    def get_blue_blocks(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")   
        # Convert BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        hsv_img = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
        
        mask_blue = cv2.inRange(hsv_img, self.lower_light_blue, self.upper_light_blue)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_blocks = []
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 100:
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    blue_blocks.append((cx, cy, 2)) # 1 for red, 2 for blue
        return blue_blocks

    def check_winner(self, board):
        # 检查每行、每列以及两个对角线
        for i in range(3):
            if board[i, 0] == board[i, 1] == board[i, 2] != 0:
                return board[i, 0]
            if board[0, i] == board[1, i] == board[2, i] != 0:
                return board[0, i]

        if board[0, 0] == board[1, 1] == board[2, 2] != 0 or \
        board[0, 2] == board[1, 1] == board[2, 0] != 0:
            return board[1, 1]

        return None  # 没有获胜者

    def get_grid_position(self, x, y, width=640, height=480):
        # 划分九宫格
        grid_x = x // (width // 3)
        grid_y = y // (height // 3)
        return grid_x, grid_y

    def update_board(self, blocks, board):
        for block in blocks:
            x, y, player = block  # 假设block包含了x坐标、y坐标和玩家标识
            grid_x, grid_y = self.get_grid_position(x, y)
            board[grid_y, grid_x] = player
            


    def choose_empty_place(self, board):
        # Choose a empty place to place chess
        print(f"self.board:\n {self.board}")
        zero_indices = np.argwhere(board == 0)
        chosen_index = random.choice(zero_indices)
        return chosen_index


    def stage_switch(self, msg):
        
        print(f"set to init position")
        req = set_positionRequest()
        req.type = 1
        req.value = self.init_observe_pose_joint
        self.move_to_goal_client(req)
        self.state = "empty" # Indicate that arm is at ready position with empty gripper
        while(self.state != "finish" and self.block_num < 5):
            print(f"start grasping")


            # This part make the arm grasp the blcok and move back to observe place
            grasp_pose = copy.deepcopy(self.init_grasp_pose)
            grasp_pose[1] += 0.06 * self.block_num
            pre_grasp_pose = copy.deepcopy(grasp_pose)
            pre_grasp_pose[2] += 0.05
            req = set_positionRequest()
            req.type = 2
            req.value = pre_grasp_pose
            res = self.move_to_goal_client(req)
            print(f"res1: {res}")

            req.value = grasp_pose
            res = self.move_to_goal_client(req)
            print(f"res2: {res}")

            req.value = [1]
            res = self.move_to_goal_client(req)
            print(f"res3: {res}")

            req.value = pre_grasp_pose
            res = self.move_to_goal_client(req)
            print(f"res4: {res}")


            req.type = 1
            req.value = self.init_observe_pose_joint
            res = self.move_to_goal_client(req)
            time.sleep(0.5)
            print(f"res5: {res}")

            # The part below make the arm place the block and return to the observe place
            # Randomly choose place to place
            index =  self.choose_empty_place(self.board)
            print(f"index: {index}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n")

            place_pose = copy.deepcopy(self.middle_place_pose)
            # x axis part
            place_pose[0] += (index[1] - 1) * 0.12
            # y axis part
            place_pose[1] += (1 - index[0]) * 0.12

            pre_place_pose = copy.deepcopy(place_pose)
            pre_place_pose[2] += 0.05

            req.type = 2
            req.value = pre_place_pose
            res = self.move_to_goal_client(req)
            


            req.value = place_pose
            res = self.move_to_goal_client(req)
            

            req.value = [0]
            res = self.move_to_goal_client(req)
            self.block_num += 1
            

            req.value = pre_place_pose
            res = self.move_to_goal_client(req)
            

            req.type = 1
            req.value = self.init_observe_pose_joint
            res = self.move_to_goal_client(req)
            

            winner = self.check_winner(self.board)
            print(f"winner: {winner}")
            print(f"self.state: {self.state}")
            print(f"self.block_num: {self.block_num}")
            if winner is not None:
                break

        winner = self.check_winner(self.board)
        print(f"winner: {winner}")
        self.block_num = 0

if __name__ == '__main__':
    try:
        node = Node1()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

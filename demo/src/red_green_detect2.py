#!/usr/bin/env python3

import random
import rospy
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np


blocks = [[167, 68], [301, 68], [435, 68],\
          [162, 193], [296, 193], [430, 193],\
          [162, 318], [300, 318], [430, 318]]

win_pattern = [{0, 1, 2}, {3, 4, 5}, {6, 7, 8},\
               {0, 3, 6}, {1, 4, 7}, {2, 5, 8},\
               {0, 4, 8}, {2, 4, 6}]

blue = [0, 0, 0, 0, 0, 0, 0, 0, 0] # color of robot arm player
red = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # color of human player
nonoccupied = [0, 1, 2, 3, 4, 5, 6, 7, 8]

random_block = -1



class Node1:
    def __init__(self):
        rospy.init_node('node_1', anonymous = True)

        # Create a CV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        # Publish the start_cmd
        self.pub = rospy.Publisher("/start_cmd", Int32, queue_size=40)
        self.firsttime=0
        self.last_time = None
        self.count = 17

        self.blue_positions = []
        self.red_positions = []

        self.pick_and_place_flag = 1 # 1 for pick, -1 for place
        
    def check_winner(self):
        
        print("check winner !")
        for i in range(9):
            if blue[i] == 1:
                self.blue_positions.append(i)
            if red[i] == 1:
                self.red_positions.append(i)
        B = set(self.blue_positions)
        R = set(self.red_positions)
        for i in range(8):
            if win_pattern[i].issubset(B):
                print("Robot arm wins!")
                return True
            elif win_pattern[i].issubset(R):
                print("Human wins!")
                return True
            elif (i == 7)and(self.count == 12):
                print("Nobody wins~")
                return True
            else:
                pass
        return False
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert BGR to RGB
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Process the image (add your image processing logic here)
            if (self.last_time is None) or (time.time() - self.last_time > 30):
                
                
                print(f"self.count: {self.count} self.pick_and_place_flag: {self.pick_and_place_flag}")
                if self.count >= 13 and self.pick_and_place_flag == 1:
                    print(f"i am here: {self.count}")
                    self.pub.publish(3)
                    self.pub.publish(self.count)
                    self.count -= 1
                    self.last_time = time.time()
                
                elif self.pick_and_place_flag == -1:
                    self.detect_red_and_light_blue(cv_image_rgb)
                    if self.check_winner():
                        print(f"someone win")
                        self.pub.publish(3)
                        self.pick_and_place_flag = 0
                
                self.pick_and_place_flag *= -1

        except Exception as e:
            rospy.logerr("Error processing image: {}".format(str(e)))

    def detect_red_and_light_blue(self, cv_image_rgb):
        # This function will detect who wins and then pub msg to place the object.                           
        self.last_time = time.time()
        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Define the range of lighter blue color in HSV
        lower_light_blue = np.array([90, 50, 50])
        upper_light_blue = np.array([130, 255, 255])

        # Threshold the image to get a binary mask of the lighter blue regions
        mask_light_blue = cv2.inRange(hsv, lower_light_blue, upper_light_blue)

        # Find contours in the binary masks
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_light_blue, _ = cv2.findContours(mask_light_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process contours for red color
        for contour in contours_red:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Set a threshold for the contour area to filter out small noise
            if area > 100:
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Display the centroid position for red
                    for i in range(9):

                        if (cx > blocks[i][0]) and (cx < (blocks[i][0] + 134)):
                            if(cy > blocks[i][1]) and (cy < (blocks[i][1] + 125)):
                                red[i] = 1
                                
                                element_to_remove = i

                                if element_to_remove in nonoccupied:
                                    nonoccupied.remove(element_to_remove)
                                    print("Element removed:", element_to_remove)

                    # Draw a circle around the centroid
                    cv2.circle(cv_image_rgb, (cx, cy), 5, (0, 255, 0), -1)

        # Process contours for light blue color
        for contour in contours_light_blue:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Set a threshold for the contour area to filter out small noise
            if area > 100:
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Display the centroid position for light blue
                    for i in range(9):
                        if (cx > blocks[i][0]) and (cx < (blocks[i][0] + 134)):
                            if(cy > blocks[i][1]) and (cy < (blocks[i][1] + 125)):
                                blue[i] = 1

                                element_to_remove = i

                                if element_to_remove in nonoccupied:
                                    nonoccupied.remove(element_to_remove)
                                    print("Element removed:", element_to_remove)
                    # Draw a circle around the centroid
                    cv2.circle(cv_image_rgb, (cx, cy), 5, (0, 255, 0), -1)

        # Display the original image with the detected centroids (optional)
        cv2.imshow('Original Image', cv_image_rgb)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

        if len(nonoccupied) == 1:
            print(f"matt: {len(nonoccupied)}") 
            
            self.pub.publish(nonoccupied[0] + 4)
        elif len(nonoccupied) > 1:
            print(f"len(nonoccupied): {len(nonoccupied)}") 
            random_block = random.randint(0, len(nonoccupied) - 1)
            print(nonoccupied[random_block])
            if self.firsttime==0:
                random_block=4
                self.firsttime+=1
            
            self.pub.publish(nonoccupied[random_block] + 4)



if __name__ == '__main__':
    try:
        node = Node1()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
from demo.srv import set_position, set_positionRequest


class PeopleDetectorNode:
    def __init__(self):
        rospy.init_node('people_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.move_to_goal_client = rospy.ServiceProxy("/move_to_goal", set_position)
        self.start_sub = rospy.Subscriber("/face_detct_start_cmd", Int32, self.move_to_people)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Perform people detection (you can replace this with your detection algorithm)
            # For example, using Haar cascades
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.3, minNeighbors=5)

            # Draw rectangles around detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                self.center_x = x + w // 2
                self.center_y = y + h // 2


            # Display the result
            cv2.imshow('People Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def move_to_people(self, msg):
        srv = set_positionRequest()  # The "srv" is the command that send to server in service
        # Back to original pose at first
        srv.type = 1
        srv.value = [0, 0, 0, 0, 0, 0]
        self.move_to_goal_client(srv)
        
        # First move to the cup
        srv.type = 1
        srv.value = [0.00851026442140271, 0.0545890599055239, 1.5253956886376072, -1.5231487796259073, 1.6349954201387282, -0.009822324827889889]
        self.move_to_goal_client(srv)
        
        # Second, close the gripper to grasp the cup
        srv.value = [1.0]
        self.move_to_goal_client(srv)

        # determine where the person is
        srv.type = 1
        if self.center_x < 210:
            print(f"left")
            srv.value = [-1.3091732402599103, 1.0618583195765112, 0.1083382836389129, -1.1581617532822208, 1.4308209793744953, 0.027964052811997073]
        elif self.center_x < 430:
            print(f"middle")
            srv.value = [-1.5699129562613172, 1.1719260350601863, 0.0003234907186233999, -1.072921823114, 1.5934564987201083, 9.696273856980042e-06]
        else:
            print(f"right")
            srv.value = [-1.1371197180353412, 0.9273981790365823, 0.7386911115674206, -1.5693710029794863, 1.2249883213575246, -0.0]
        self.move_to_goal_client(srv)

        # Flip the cup
        srv.value[5] = -1.57
        self.move_to_goal_client(srv)

        # Return to the place where cup was placed
        srv.type = 1
        srv.value = [0.00851026442140271, 0.0545890599055239, 1.5253956886376072, -1.5231487796259073, 1.6349954201387282, -0.009822324827889889]
        self.move_to_goal_client(srv)

        # Release the cup
        srv.type = 1
        srv.value = [0.]
        self.move_to_goal_client(srv)

        # Back to original pose
        srv.type = 1
        srv.value = [0., 0., 0., 0., 0., 0.]

        self.move_to_goal_client(srv)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PeopleDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

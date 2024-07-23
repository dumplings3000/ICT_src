#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32

class RedCupDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        rospy.init_node('red_cup_detector', anonymous=True)
        self.pub = rospy.Publisher('cup_position', Int32, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

    def detect_red_cup(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            return x + w // 2
        else:
            return None

    def capture_webcam_and_publish(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if ret:
                cup_x = self.detect_red_cup(frame)
                print(f"cup_x: {cup_x}")
                if cup_x is not None:
                    
                    width = frame.shape[1]
                    if cup_x < width / 3:
                        cup_position = 0
                    elif cup_x < 2 * width / 3:
                        cup_position = 1
                    else:
                        cup_position = 2

                    self.pub.publish(cup_position)

            self.rate.sleep()

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        red_cup_detector = RedCupDetector()
        red_cup_detector.capture_webcam_and_publish()
        red_cup_detector.release()
    except rospy.ROSInterruptException:
        pass

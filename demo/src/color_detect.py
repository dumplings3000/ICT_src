#!/usr/bin/env python3

import rospy
import cv2
import open3d as o3d
import numpy as np
import cv_bridge
import tf2_ros
from tf.transformations import quaternion_matrix
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from demo.srv import set_position, set_positionRequest

class Node1:
    def __init__(self):
        rospy.init_node('detect_color', anonymous = True)

        # Create a CV bridge
        self.cv_bridge = cv_bridge.CvBridge()

        # Subscribe to the image topic
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=1, slop=2)
        self.ts.registerCallback(self.topic_callback)
        self.move_to_goal_client = rospy.ServiceProxy("/move_to_goal", set_position)
        rospy.Subscriber('/start', Int32, self.decision_callback)

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)  # Create a tf listener

        # color boundaries
        # Red
        self.lower_red = np.array([0, 100, 100])
        self.upper_red = np.array([10, 255, 255])
        # Green
        self.lower_green = np.array([25, 20, 75])
        self.upper_green = np.array([50, 255, 255])
        # Blue
        self.lower_blue = np.array([90, 120, 50])
        self.upper_blue = np.array([130, 230, 255])

        self.color_to_point = {
            'red': (10, 20),
            'blue': (30, 40),
            'green': (50, 60)
            }

        # target parameters
        self.mask_list = []  # list of masks of detected objects
        self.target_center = None  # center of target object
        self.target_pointcloud = None  # point cloud of target object
        self.target_color = None  # color of target object
        self.target_pose = [10.0] * 6
        
        self.o3d_camera_intrinsic = None  # camera intrinsic parameters
        # self.init_observe_pose_joint = [-1.6346607939477598, 0.11474589083353559, -1.36259891488799, -0.32196475748740616, -1.5711840830496122, -0.08652954928133474]
        # self.init_grasp_pose = [-0.45866827392578126, 0.43749786376953126, 0.19912411499023438, -3.14, 0, -1.57]
        self.init_observe_pose_cartesian = [-1.5746906672569194, 0.21244009383951182, -1.645535146468826, -0.13358555843794687, -1.5712132446635485, -0.003277340392985478]
        self.init_grasp_pose_cartesian = [-0.45866827392578126, 0.43749786376953126, 0.19912411499023438, -3.14, 0, -1.57]
        self.mode = 0 # 0 is observe, 1 is pre-grasp, 2 is grasp
        self.flag = False
        self.done = False

    def descision_callback(self, msg):
        self.mode = msg.data
        grasp_pose = [0,0,0,-3.14, 0, -1.57]
        while(1): 

            if self.mode == 0:# observe
                self.request(1, self.init_observe_pose_cartesian, 2)
                self.flag = True
                if self.done == True:
                    self.done = False
                    self.mode = 1

            elif self.mode == 1:# pre-grasp
                pre_grasp_pose = self.target_pose
                pre_grasp_pose[2] = pre_grasp_pose[2] + 0.02
                self.request(1, pre_grasp_pose, 2)
                self.mode = 2
                
            elif self.mode == 2:# grasp
                self.request(0, self.target_pose, 2)
                self.request(0, self.init_grasp_pose_cartesian, 2)
                self.mode = 3

            elif self.mode == 3:# pre-place
                grasp_pose[0],grasp_pose[1] = self.color_to_point[self.target_color]
                grasp_pose[2] = self.target_center[2] + 0.04
                self.request(0, grasp_pose, 2)
                self.mode = 4

            elif self.mode == 4:# place
                grasp_pose[0],grasp_pose[1] = self.color_to_point[self.target_color]
                grasp_pose[2] = self.target_center[2] + 0.02
                self.request(1, grasp_pose, 2)
                self.mode = 0

    def topic_callback(self, rgb_msg, depth_msg):
        if self.flag == True:
            try:
                rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                ori_H, ori_W, _ = rgb_img.shape
                T = self.get_transform(depth_msg)
                self.o3d_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                                        640, 480,
                                        617.0441284179688*640/ori_W,
                                        617.0698852539062*480/ori_H,
                                        322.3338317871094, 238.7687225341797)
                self.mask_list = self.detect_color(rgb_img)
                self.target_center, self.target_color = self.deside_target_object(rgb_img, depth_img, T)
                self.target_pose = [self.target_center[0], self.target_center[1], self.target_center[2] + 0.02 , -3.14, 0, -1.57]
                self.flag = False
                self.done = True

            except Exception as e:
                rospy.logerr("Error processing image: {}".format(str(e)))
    
    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # mask create
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)

        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        mask_list = []
        for contour in blue_contours:
            mask = np.zeros_like(blue_mask)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            mask_list.append((mask, 'blue'))

        for contour in red_contours:
            mask = np.zeros_like(red_mask)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            mask_list.append((mask, 'red'))

        for contour in green_contours:
            mask = np.zeros_like(green_mask)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            mask_list.append((mask, 'green'))

        return mask_list
    
    def get_transform(self, depth_msg):
        try:
            """
            convert camera_color_optical_frame to camer_link
            """
            transform_stamped = self.tf_buffer.lookup_transform('base', depth_msg.header.frame_id, rospy.Time(0))
            trans = np.array([transform_stamped.transform.translation.x,
                              transform_stamped.transform.translation.y,
                              transform_stamped.transform.translation.z])
            quat = np.array([transform_stamped.transform.rotation.x,
                            transform_stamped.transform.rotation.y,
                            transform_stamped.transform.rotation.z,
                            transform_stamped.transform.rotation.w])
            T = quaternion_matrix(quat)
            T[:3, 3] = trans
            return T
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform point cloud: {}".format(e))
            return
        
    def deside_target_object(self, rgb_img, depth_img, T):
        pointcloud_list = []
        for i in range(self.mask_list):
            mask = np.array(self.mask_list[i][0]).astype(int)
            un_depth_img = unnormalize_depth(depth_img)
            un_depth_img[np.logical_not(mask)] = 0
            kernel = np.ones((3, 3), np.uint8)
            un_depth_img = cv2.erode(un_depth_img, kernel, iterations=1)
            o3d_rgb_img = o3d.geometry.Image(rgb_img)
            o3d_depth_img = o3d.geometry.Image(un_depth_img)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb_img, o3d_depth_img)

            o3d_pc = o3d.geometry.PointCloud.create_from_rgbd_image(
                                                rgbd_image, self.o3d_camera_intrinsic)
            sampled_pc = np.asarray(o3d_pc.points)
            sampled_o3d_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sampled_pc))
            pcd_filtered, _ = sampled_o3d_pc.remove_statistical_outlier(nb_neighbors=40, std_ratio=1.5)
            pointcloud_list.append(pcd_filtered)

        dis_list = [np.linalg.norm(x.get_center()) for x in pointcloud_list]  # get distance between point cloud and origina
        index = np.argmin(dis_list)  # choose the closet one as target
        target_pointcloud = pointcloud_list[index].transform(T)  # convert the target point cloud to world frame
        target_center = target_pointcloud.get_center()  # get point center of target point cloud in world frame
        target_color  = self.mask_list[index][1]  # get color of target object

        return target_center, target_color
    
    def request(self, grasp, pose, type):
        req = set_positionRequest()
        req.type = type
        req.value = pose
        res = self.move_to_goal_client(req)
        print(f"pose control: : {res}")

        req.value = grasp
        res = self.move_to_goal_client(req)
        print(f"gripper control: {res}")


if __name__ == '__main__':
    try:
        node = Node1()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

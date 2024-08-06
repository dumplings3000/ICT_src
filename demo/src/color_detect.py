#!/usr/bin/env python3

import rospy
import cv2
import open3d as o3d
import numpy as np
import cv_bridge
import tf2_ros
from tf.transformations import quaternion_matrix
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from demo.srv import set_position, set_positionRequest
from tm_msgs.msg import FeedbackState

class Node1:
    def __init__(self):
        rospy.init_node('detect_color', anonymous = True)

        # Create a CV bridge
        self.cv_bridge = cv_bridge.CvBridge()

        # Subscribe to the image topic
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.camera_info_topic = rospy.get_param("~camera_info", "/camera/color/camera_info")
        rospy.wait_for_service('/move_to_goal')
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
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])

        self.color_to_point = {
            'red': [0.4544473571777344, 0.6317485961914063],
            'blue': [0.4544473571777344, 0.7317485961914063],
            'green': [0.4544473571777344, 0.5317485961914063]
            }

        # target parameters
        self.n_instances = 0  # number of detected objects
        self.mask_list = []  # list of masks of detected objects
        self.target_center = None  # center of target object
        self.target_color = None  # color of target object
        self.target_pose = [10.0] * 6
        
        self.o3d_camera_intrinsic = None  # camera intrinsic parameters
        self.init_observe_pose = [-1.522882330358845, -0.13996572675096333, -1.4681066993742042, 0.03937656777153009, -1.5694194725113255, 0.04696874901902001]
        self.init_grasp_pose = [-1.522882330358845, -0.13996572675096333, -1.4681066993742042, 0.03937656777153009, -1.5694194725113255, 0.04696874901902001]
        self.mode = 0 # 0 is observe, 1 is pre-grasp, 2 is grasp
    
    def decision_callback(self, msg):
        self.mode = msg.data
        place_pose = [0, 0, 0, -3.14, 0, 3.14]
        while(1): 
            if self.mode == 0:# observe
                self.request(0, self.init_observe_pose, 1)
                print("done mode0")
                self.get_target_pose()
                if self.n_instances == 0:
                    break
                self.mode = 1

            elif self.mode == 1:# pre-grasp
                pre_grasp_pose = self.target_pose.copy()
                pre_grasp_pose[2] = self.target_pose[2] + 0.04
                print("pre_grasp_pose", pre_grasp_pose)
                self.request(0, pre_grasp_pose, 2)
                print("done mode 1")
                self.mode = 2
                
            elif self.mode == 2:# grasp
                self.request(1, self.target_pose, 2)
                print("done grasp_pose")
                self.request(1, self.init_grasp_pose, 1)
                print("done mode 2")
                self.mode = 3

            elif self.mode == 3:# pre-place
                place_pose[0],place_pose[1] = self.color_to_point[self.target_color]
                place_pose[2] = self.target_pose[2].copy() + 0.04
                self.request(1, place_pose, 2)
                print("done mode 3")
                self.mode = 4

            elif self.mode == 4:# place
                place_pose[0],place_pose[1] = self.color_to_point[self.target_color]
                place_pose[2] = self.target_pose[2].copy()
                self.request(0, place_pose, 2)
                print("done mode 4")
                self.mode = 5

            elif self.mode == 5:
                place_pose[0],place_pose[1] = self.color_to_point[self.target_color]
                place_pose[2] = self.target_pose[2].copy() + 0.04
                self.request(0, place_pose, 2)
                self.color_to_point[self.target_color][0] -= 0.06
                print("done mode 5")
                self.mode = 0
            else:
                break

    def get_target_pose(self):
        try:
            self.target_pose = [0] * 6
            while all(x == 0 for x in self.target_pose[:2]):
                self.n_instances = 0
                while self.n_instances == 0:
                    rgb_msg = rospy.wait_for_message(self.rgb_topic, Image, timeout=1)
                    depth_msg = rospy.wait_for_message(self.depth_topic, Image, timeout=1)
                    rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                    self.mask_list = self.detect_color(rgb_img)
                    print("finised detect color")
                depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
                ori_H, ori_W, _ = rgb_img.shape
                self.H, self.W, _= rgb_img.shape
                camera_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
                self.K = np.array(camera_info.K)
                # self.o3d_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                #                         self.W, self.H,
                #                         self.K[0] * self.W / ori_W,
                #                         self.K[4] * self.H / ori_H, 
                #                         self.K[2], self.K[5])
                self.o3d_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                                        640, 480,
                                        617.0441284179688*640/ori_W,  
                                        617.0698852539062*480/ori_H,
                                        322.3338317871094, 238.7687225341797)
                T = self.get_transform(depth_msg)
                self.target_center, self.target_color = self.decide_target_object(rgb_img, depth_img, T)
                print("finised decide target object")
                self.target_pose = [self.target_center[0], self.target_center[1], self.target_center[2] + 0.11 , -3.14, 0, 3.14]
            print("target_pose", self.target_pose)

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

        self.n_instances = len(mask_list)

        print("n_instances", self.n_instances)

        # # Create an output image
        # output_img = img.copy()

        # # Display each mask on the image
        # for mask, color in mask_list:
        #     if color == 'blue':
        #         output_img[mask == 255] = [255, 0, 0]  # Blue color
        #     elif color == 'red':
        #         output_img[mask == 255] = [0, 0, 255]  # Red color
        #     elif color == 'green':
        #         output_img[mask == 255] = [0, 255, 0]  # Green color
        
        # cv2.imshow('Detected Colors', output_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

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
        
    def decide_target_object(self, rgb_img, depth_img, T):
        pointcloud_list = []
        dis_list = []  
        for i in range(self.n_instances):
            mask = np.array(self.mask_list[i][0]).astype(int)
            depth_img_copy = depth_img.copy()
            depth_img_copy[np.logical_not(mask)] = 0
            kernel = np.ones((3, 3), np.uint8)
            depth_img_copy = cv2.erode(depth_img_copy, kernel, iterations=1)
            o3d_rgb_img = o3d.geometry.Image(rgb_img)
            o3d_depth_img = o3d.geometry.Image(depth_img_copy)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb_img, o3d_depth_img)

            o3d_pc = o3d.geometry.PointCloud.create_from_rgbd_image(
                                                rgbd_image, self.o3d_camera_intrinsic)
            sampled_pc = np.asarray(o3d_pc.points)
            sampled_o3d_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sampled_pc))
            pcd_filtered, _ = sampled_o3d_pc.remove_statistical_outlier(nb_neighbors=40, std_ratio=1.5)
            pcd_filtered = pcd_filtered.transform(T)
            pointcloud_list.append(pcd_filtered)

            # print("the number of i :",i+1)
            # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
            # o3d.visualization.draw_geometries([pcd_filtered, axis])
        
        # combined_pcd = o3d.geometry.PointCloud()
        # for pcd in pointcloud_list:
        #     combined_pcd += pcd
        # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # o3d.visualization.draw_geometries([combined_pcd, axis])

        dis_list = [np.linalg.norm(x.get_center()) for x in pointcloud_list]  # get distance between point cloud and origin
        print("distance list", dis_list)
        index = np.argmin(dis_list)  # choose the closet one as target
        print("target index", index)

        # target_pointcloud = pointcloud_list[index].transform(T)  # convert the target point cloud to world frame
        target_pointcloud = pointcloud_list[index]
        target_center = target_pointcloud.get_center()  # get point center of target point cloud in world frame
        print("target_center", target_center)
        T_inv = np.linalg.inv(T)
        two_d_target_pointcloud = target_pointcloud.transform(T_inv)
        two_d_target_center = two_d_target_pointcloud.get_center()
        print("two_d_target_center", two_d_target_center)
        target_center[0] -= 0.025 
        if two_d_target_center[0] > 0.05 :
            target_center[0] = target_center[0]
        elif two_d_target_center[0] < -0.05 :
            target_center[0] = target_center[0]
        target_color  = self.mask_list[index][1]  # get color of target object
        print("target_color", target_color)
        # # 顯示點雲和座標軸
        # print("after tranform target_center", target_center)
        # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # o3d.visualization.draw_geometries([target_pointcloud, axis])

        return target_center, target_color
    
    def request(self, grasp, pose, type):
        req = set_positionRequest()
        req.type = type
        req.value = pose
        res = self.move_to_goal_client(req)
        print(f"mode : {self.mode} ,pose control: {res.response}")

        req.value = [grasp]
        res = self.move_to_goal_client(req)
        print(f"mode : {self.mode} ,gripper control: {res}")
    
if __name__ == '__main__':
    try:
        node = Node1()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

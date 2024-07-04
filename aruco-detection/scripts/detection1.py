import numpy as np
import cv2
import sys
import time
import os
import struct
import json
import pyrealsense2 as rs

from scipy.stats import trim_mean

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from tf.transformations import euler_matrix, euler_from_matrix
from scipy.spatial.transform import Rotation as R
from transforms3d.euler import euler2mat, mat2euler

class aruco_detect():
    def __init__(self):

        self.ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
                        }
                
        self.aruco_type = "DICT_6X6_250"
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.intrinsic_camera = np.array(((606.7596435546875, 0.0, 330.67840576171875), (0.0, 606.5953369140625, 249.7525634765625), (0.0, 0.0, 1.0)))
        self.distortion = np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        self.bridge = CvBridge()
        self.cv_color_image = None
        self.cv_depth_image = None

        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=True)
        # Subscribe to color and depth images
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        rospy.loginfo("Image subscriber node started, subscribing to /camera/color/image_raw and /camera/depth/image_raw")
        self.color_frame = None
        self.depth_frame = None
        self.pose= None
        self.offsets = [0,0,0,0]

    def color_callback(self, msg):
        try:
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
    def depth_callback(self, msg):
        try:
            # Note: Change encoding to suit your depth format, e.g., "16UC1" for 16-bit or "passthrough" to use sensor's encoding
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")#changes here can be done
        except CvBridgeError as e:
            rospy.logerr(e)        
    def get_T_base_cam(self):
        try:
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            time.sleep(1.0)
            print("tfBuffer connection succeeded")
        except Exception as e:
            print("tfBuffer connection failed.", e)
            return
        
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform("world", "camera_mount", rospy.Time()) # Check if transform exists
                # print(f"translation : {trans}")
            except Exception as e:
                print(f"the error is : {e}")
                continue
            # trans.transform.translation.x = self.offsets[0] + trans.transform.translation.x 
            # trans.transform.translation.y = self.offsets[1] + trans.transform.translation.y 
            q = np.array([trans.transform.rotation.x, trans.transform.rotation.y,
                        trans.transform.rotation.z, trans.transform.rotation.w])
            rot = R.from_quat(q)
            r = rot.as_dcm()
            t = np.array([trans.transform.translation.x,
                        trans.transform.translation.y, trans.transform.translation.z])
            T = np.vstack((
                np.hstack((r, t.reshape(3, 1))),
                np.array([0, 0, 0, 1.0])
            ))
            #print(f't:{T}')
            return T.tolist()

    def save_marker_pose(self, markerId, pose, file_path):
        data = {}
        # Ensure the file_path directory exists
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
        # Check if the file already exists and load its content
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                try:
                    data = json.load(file)
                except json.JSONDecodeError:
                    print("Warning: Empty or corrupted JSON file. Starting fresh.")
        
        # Convert markerId to string to ensure compatibility with JSON keys
        markerId_str = str(markerId)
        
        # Prepare the data to save. Ensure pose and depth are converted to strings if necessary
        new_data = json.dumps(pose)
        
        # Check if the data for this markerId already exists and is different
        if markerId_str not in data or data[markerId_str] != new_data:
            data[markerId_str] = new_data
            # Write the updated data back to the file
            with open(file_path, 'w') as file:
                json.dump(data, file, indent=4)
        print("saved")

    def aruco_pose_estimation(self,image,depth_fr):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type])
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
            
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                #There are two variants one using a inverse of intrinisic matrix and directly using the intrinisic matrix
                #using the intrinisic matrix: features-the zeroes are from top lept of the image
                #Using the inverse of Intrinsic matrix: things depend how you multiply your center vairable with K_inv
                #cX = cX -640/2
                #cY = cY -480/2
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,2, (0, 255, 0), 2)
                cv2.imshow("Aruco_detection",image)
                center = np.array([cX,cY,1.0])
                print(center)
                # K_inv = np.linalg.inv(self.intrinsic_camera)
                pose = center@self.intrinsic_camera
                print(pose)
                # pose1 = K_inv@center
                # print(pose1)
                depth_point = self.depth(depth_fr,cX,cY)
                print(f'depth:{depth_point}')
                pose = pose*depth_point[0]
                pose = pose/100
                print(pose)
                position = [pose[0],pose[1],pose[2]]
                self.save_marker_pose(markerID, position,"/home/bhavya/catkin_ws/src/aruco-detection/config/marker_poses.json")  

    def depth(self,depth_fr,cX,cY):
        depth_frame_copy = np.copy(depth_fr)
        depth = 0
        depth = depth_frame_copy[cY,cX] 
        print(depth)       
        return depth
    
    def _run(self):
        try:
            while not rospy.is_shutdown():
                if self.cv_color_image is not None and self.cv_depth_image is not None:
                    cv2.imshow("Color Feed", self.cv_color_image)
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.cv_depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    self.aruco_pose_estimation(self.cv_color_image,depth_colormap)
            
                cv2.waitKey(1)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            cv2.destroyAllWindows()

if __name__=="__main__":
    aruco_detect()._run()
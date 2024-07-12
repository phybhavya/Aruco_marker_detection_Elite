#!/usr/bin/env python3
import numpy as np
import cv2
import os
import json
import pyrealsense2 as rs
from scipy.stats import trim_mean
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_matrix, euler_from_matrix
from scipy.spatial.transform import Rotation as R
from transforms3d.euler import euler2mat, mat2euler
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
import tf

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
        #self.intrinsic_camera = np.array(((607.379638671875, 0.0, 323.45916748046875), (0.0, 607.0968627929688, 245.50621032714844), (0.0, 0.0, 1.0)))
        self.distortion = np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        self.file_path = "config/marker_poses.json"
        self.bridge = CvBridge()
        self.cv_color_image = None
        self.cv_depth_image = None
        self.aruco_marker_name = "aruco_marker"
        rospy.init_node('image_subscriber', anonymous=True)
        # Subscribe to color and depth images
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        rospy.loginfo("Image subscriber node started, subscribing to /camera/color/image_raw and /camera/depth/image_raw")
        self.color_frame = None
        self.depth_frame = None
        self.offsets = [0,0,0,0]
        self.tfbroadcaster = TransformBroadcaster()
    def color_callback(self, msg):
        try:
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, msg):
        try:
            # Note: Change encoding to suit your depth format, e.g., "16UC1" for 16-bit or "passthrough" to use sensor's encoding
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_aruco(self,image,depth_frame,cam_matrix,dis_matrix):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type])
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,cameraMatrix=cam_matrix,distCoeff = dis_matrix)
            
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
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    2, (0, 255, 0), 2)
                
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.03,cam_matrix,dis_matrix)
                cv2.aruco.drawDetectedMarkers(image,[markerCorner]) 
                cv2.drawFrameAxes(image,cam_matrix, dis_matrix, rvec, tvec, 0.03) 
                pose = tvec[0][0]
                pos = np.array([pose[0],pose[1],pose[2]])
                print (pose)
                print(f'tvec:{tvec}')
                distance, oreintation = self.orientation(tvec,rvec)
                print(f'distance:{distance}')
                oreintation = np.array(oreintation)
                oreintation = oreintation[0]
                print(oreintation[0])
                position = [pos[0],pos[1],pos[2],oreintation[0],oreintation[1],oreintation[2]]
                self.save_marker_pose(markerID, position,"/home/bhavya/catkin_ws/src/aruco-detection/config/marker_poses.json")#src/aruco-detection/config/marker_poses.json
                cv2.imshow('Estimated Pose', image)
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'camera_depth_optical_frame'
                t.child_frame_id = self.aruco_marker_name
            
                # Store the translation (i.e. position) information
                t.transform.translation.x = tvec[0][0][0]
                t.transform.translation.y = tvec[0][0][1]
                t.transform.translation.z = tvec[0][0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                t.transform.rotation.x = quat[0] 
                t.transform.rotation.y = quat[1] 
                t.transform.rotation.z = quat[2] 
                t.transform.rotation.w = quat[3] 
        
                # Send the transform
                self.tfbroadcaster.sendTransform(t)    

    def orientation(self,tvecs,rvecs):
        rot =[]
        distance = np.linalg.norm(tvecs)
        round_rvecs = np.round(rvecs[0],decimals=4)
        print(f"round_rvecs:{round_rvecs}")
        angles = cv2.Rodrigues(round_rvecs)[0]
        R = np.array(angles)
        # Extract the pitch (Y-axis rotation) angle
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
        # pitch1 = np.arcsin(-R[2,0])

        # Extract the yaw (Z-axis rotation) angle
        yaw = np.arctan2(R[1, 0], R[0, 0])
        # Extract the roll (X-axis rotation) angle
        roll = np.arctan2(R[2, 1], R[2, 2])
        print(f"roll: {roll}    pitch: {pitch}    yaw: {yaw}")
        rot.append([roll,pitch,yaw])

        return distance,rot 
    
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


    def _run(self):

        try:
            while not rospy.is_shutdown():
                if self.cv_color_image is not None and self.cv_depth_image is not None:
                    cv2.imshow("Color Feed", self.cv_color_image)
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.cv_depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    self.detect_aruco(self.cv_color_image,depth_colormap,self.intrinsic_camera,self.distortion)
                
                cv2.waitKey(1)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            cv2.destroyAllWindows()
            


if __name__=="__main__":
    aruco_detect()._run()
    
 
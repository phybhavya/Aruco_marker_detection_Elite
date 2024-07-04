#!/usr/bin/env python3
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
        #self.intrinsic_camera = np.array(((388.2025146484375, 0.0, 321.3910217285156), (0.0, 388.2025146484375, 238.2572784423828), (0.0, 0.0, 1.0)))
        #self.intrinsic_camera = np.array(((607.379638671875, 0.0, 323.45916748046875),(0.0, 607.0968627929688, 245.50621032714844),(0.0, 0.0, 1.0)))
        self.distortion = np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        self.file_path = "config/marker_poses.json"
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
        self.offsets = [0,0,0,0]

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
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    2, (0, 255, 0), 2)
                
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.027,cam_matrix,dis_matrix)
                cv2.aruco.drawDetectedMarkers(image,[markerCorner]) 
                cv2.drawFrameAxes(image,cam_matrix, dis_matrix, rvec, tvec, 0.03) 
                pose = tvec[0][0]
                print(pose)
                print(f'tvec:{tvec}')
                distance, oreintation = self.orientation(tvec,rvec)
                print(f'distance:{distance}')
                # depth = self.depth(depth_frame,cX,cY)
                # print(f'depth:{depth}')
                # print("center x: {},center y : {}".format(cX,cY))
                # print("\033[91m\033[3mid no.::{}, depth :: {}\033[0m".format(markerID, depth))
                # print(f"{Colors.YELLOW}Detected Marker ID:: {markerID}, At pose of x::{pose[0]}, y::{pose[1]}, z::{pose[2]}, orientation::{oreintation}, distance::{distance}{Colors.RESET}")                
                # Ximg = [cX,cY,depth/1000]
                # Xbase = self.Ximg2Xbase(Ximg)
                T_cam_base = self.get_T_base_cam()
                T_cam_base = np.array(T_cam_base)# Initially it was initialised after the the X_base variable but imo it is wrong so changed it
                print(f'T_cam_base:{T_cam_base}')
                Xbase = T_cam_base @ np.array([pose[0], pose[1], pose[2], 1.0])
                print(f'Xbase:{Xbase}')
                #T_cam_base = np.array(T_cam_base)
                R_cam_base = np.array(T_cam_base[:3, :3])
                print(f'R_cam_base:{R_cam_base}')
                R_cam = R.from_euler('xyz', oreintation[0], degrees=False).as_dcm()
                R_base = R_cam_base @ R_cam #check this transformation
                print(f'Rbase:{R_base}')
                orientation_base = R.from_dcm(R_base).as_euler('xyz', degrees=False)
                #orientation_base[0], orientation_base[2] = orientation_base[2], -orientation_base[0]
                position = [Xbase[0],Xbase[1],Xbase[2],orientation_base[0], orientation_base[1], orientation_base[2]]
                self.save_marker_pose(markerID, position,"/home/bhavya/catkin_ws/src/aruco-detection/config/marker_poses.json")#/home/ow-labs/workspaces/robogpt/diff_seek/aruco_detection/config/marker_poses.json")
                cv2.imshow('Estimated Pose', image)
                #print(position)
                # orient = self.ort_base(oreintation)
                # # Xbase = [Xbase[0], Xbase[1], Xbase[2]]
                # Xbase = [Xbase[0], Xbase[1], Xbase[2], orient[0], orient[1], orient[2]]
                # print(f"{Colors.YELLOW}Detected Marker ID:: {markerID}, At pose of {Xbase} {Colors.RESET}")                


    def orientation(self,tvecs,rvecs):
        rot =[]
        distance = np.linalg.norm(tvecs)
        round_rvecs = np.round(rvecs[0],decimals=1)
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

    def depth(self,depth_frame,cX,cY):
        depth_frame_copy = np.copy(depth_frame)
        clm = depth_frame_copy.shape[1]
        row = depth_frame_copy.shape[0]

        
        # area = abs(xmax-xmin)*abs(ymax-ymin) # Area of bounding box

        # Getting depth at center of bounding box using depth_frame        
        depth = 0
        depth = depth_frame_copy[cY,cX]
        
        return depth/1000
    
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
               # trans = tfBuffer.lookup_transform("base_link", "camera_color_optical_frame", rospy.Time()) # Check if transform exists
                trans = tfBuffer.lookup_transform("world", "camera_mount", rospy.Time()) # Check if transform exists
                # print(f"translation : {trans}")
            except Exception as e:
                print(f"the error is : {e}")
                continue
            trans.transform.translation.x = self.offsets[0] + trans.transform.translation.x 
            trans.transform.translation.y = self.offsets[1] + trans.transform.translation.y 
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
        
    def ort_base(self,orientation):
        rot_matrix = euler2mat(*orientation[0], axes='sxyz')        
        T_cam_base = np.round(np.array(self.get_T_base_cam())[:3,:3])
        combined_matrix = np.dot(T_cam_base, rot_matrix)
        # Convert the resulting rotation matrix back to Euler angles
        ort = mat2euler(combined_matrix, axes='sxyz')
        return ort
    
    def Ximg2Xbase(self, Ximg)->np.ndarray:
            """
            Convert image coordinates to robot base coordinates using the transformation matrix and intrinsic matrix.

            Args:
                Ximg (list): Image coordinates [Ximg_x, Ximg_y, Ximg_depth/1000].
                robot_ip (str): The IP address of the robot.

            Returns:
                np.ndarray: The converted coordinates in the robot base frame as [Xbase_x, Xbase_y, Xbase_z].
            """
            try:
                T_cam_base = self.get_T_base_cam()
                Z = Ximg[2]
                Ximg = np.array([Ximg[0], Ximg[1], 1.0])
                Xcam = Z * np.linalg.inv(self.intrinsic_camera) @ Ximg
                Xbase = T_cam_base @ np.array([Xcam[0], Xcam[1], Xcam[2], 1.0])
                return Xbase[:3]
            except Exception as e:
                self.return_direct = True
                print("Error converting image coordinates to robot base: "+str(e))


    def _run(self,cam_type="stereo"):
            
        # if cam_type == "stereo":
        #     pipeline = rs.pipeline()
        #     config = rs.config()
        #     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #     pipeline.start(config)
            
            # try:
            #     while True:
            #         frames = pipeline.wait_for_frames()
            #         depth_frame = frames.get_depth_frame()
            #         color_frame = frames.get_color_frame()
            #         if not depth_frame or not color_frame:
            #             continue

            #         # Convert images to numpy arrays
            #         depth_image = np.asanyarray(depth_frame.get_data())
            #         color_image = np.asanyarray(color_frame.get_data())
            #         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            #         self.detect_aruco(color_image,depth_image,self.intrinsic_camera,dis_matrix=self.distortion)

            #         cv2.imshow('Color Feed', color_image)
            #         # cv2.imshow('Depth Feed', depth_colormap)

            #         # Break the loop when 'q' is pressed
            #         if cv2.waitKey(1) & 0xFF == ord('q'):
            #             break
            # finally:
            #     # Stop streaming
            #     pipeline.stop()
            #     cv2.destroyAllWindows()
        try:
            while not rospy.is_shutdown():
                if self.cv_color_image is not None and self.cv_depth_image is not None:
                    cv2.imshow("Color Feed", self.cv_color_image)
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.cv_depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    # cv2.imshow("Depth Feed", depth_colormap)
                    #T_cam_base = self.get_T_base_cam()
                    #print(T_cam_base)
                    self.detect_aruco(self.cv_color_image,depth_colormap,self.intrinsic_camera,self.distortion)
                
                cv2.waitKey(1)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            cv2.destroyAllWindows()
            


if __name__=="__main__":
    aruco_detect()._run()
    
 
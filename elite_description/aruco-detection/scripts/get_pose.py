import json
import os
import sys
import time
import rospy
import tf2_ros
import numpy as np
import ast
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_matrix,euler_from_matrix
import cv2
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
import tf.transformations
class TF_trans():
    """
    Convert image frame coordinates to robot base frame coordinates for a specific object.

    This class provides functionality to convert image coordinates (Ximg) to robot base coordinates (Xbase)
    for a specified object. It takes into account the transformation matrix between the robot base and the camera,
    as well as the intrinsic matrix of the camera.

    Methods:
        read_json_file(file_name: str) -> dict:
            Read data from a JSON file (detection_results.json) and return it as a dictionary.

        get_T_cam_base(robot_ip: str) -> list:
            Get the transformation matrix between the camera_color_optical_frame and the robot base_link for the specified robot IP using tf2_ros Buffer 
            (NEEDS TO HAVE THE TF BEING PUBLISHED USING RVIZ or TF BROADCASTER).

        get_ximg(object: str, robot_ip: str) -> list:
            Get the object's pose in the image frame for the specified object and robot IP.

        get_orientation(object: str, robot_ip: str) -> list or None:
            Get the object's orientation in the robot base frame for the specified object and robot IP.

        Ximg2Xbase(Ximg: list, robot_ip: str) -> np.ndarray:
            Convert image coordinates to robot base coordinates using the transformation matrix and intrinsic matrix.

        _run(object: str, robot_ip: str, include_ort: bool = False) -> list:
            Convert image coordinates to robot base coordinates for a specific object and robot IP. Returns the pose in [XYZ, RPY] format.

    """
    def __init__(self) -> None:
        rospy.init_node('image_subscriber', anonymous=True)
        self.markerpath = os.path.join(os.getcwd(),"config/marker_poses.json")
        self.inst_matrix = np.array([[606.7596435546875, 0.0, 330.67840576171875], [0.0, 606.5953369140625, 249.7525634765625], [0.0, 0.0, 1.0]])
        self.poses = None
        self.oreintation = None
        self.tfbroadcaster = TransformBroadcaster()
        self.poses1 = None

    def read_json_file(self, file_name):
        """
        Read data from a JSON file and return it as a dictionary.

        Args:
            file_name (str): The path to the JSON file to be read.

        Returns:
            dict: The data read from the JSON file as a dictionary.
        """
        with open(file_name, 'r') as f:
            marker_pose=json.load(f)
            return marker_pose
        


    def get_T_cam_base(self):
        # Check if the ROS is running
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
                trans = tfBuffer.lookup_transform("base_link","aruco_marker", rospy.Time()) # Check if transform exists
            except Exception as e:
                print(f"the error is : {e}")
                continue
            q = np.array([trans.transform.rotation.x, trans.transform.rotation.y,
                        trans.transform.rotation.z, trans.transform.rotation.w])
            t = np.array([trans.transform.translation.x,
                        trans.transform.translation.y, trans.transform.translation.z])
            rot = R.from_quat(q)
            r = rot.as_dcm()
            r2 = tf.transformations.quaternion_matrix(q)
            r3 = tf.transformations.euler_from_matrix(r)
            print(r3)
            r3_degrees = np.degrees(r3)
            print(r3_degrees)
            # print(r2)
            # print(r)
            T = np.vstack((
                np.hstack((r, t.reshape(3, 1))),
                np.array([0, 0, 0, 1.0])
            ))
            print("succeded")
            print(T)
            return T.tolist()
            # return transformed_pose
                
    def get_ximg(self,id):
        """
        Get the object's pose in the image frame for the specified object and robot IP.

        Args:
            object (str): The name of the object for which to retrieve the pose.
            robot_ip (str): The IP address of the robot.

        Returns:
            list: The object's pose in the image frame as [Ximg_x, Ximg_y, Ximg_depth].
        """        
        try:   
            data = self.read_json_file("src/aruco-detection/config/marker_poses.json")
            converted_data = {key: np.array(ast.literal_eval(value)) for key, value in data.items()}
            self.poses = converted_data["14"]
            self.poses1 = [self.poses[0],self.poses[2],self.poses[1]]
            print(self.poses1)
            self.oreintation = [self.poses[3],self.poses[4],self.poses[5]]
        except Exception as e:
            print(f"the error is : {e}")
            
    def get_orientation(self,oreintation):
        try:
            rotation_matrix = euler_matrix(oreintation[0],oreintation[1],oreintation[2],axes="sxyz")
            R_cam_base = np.round(np.array(self.get_T_cam_base())[:3,:3])
            ort_base = euler_from_matrix(R_cam_base @ rotation_matrix[:3,:3])
            return ort_base
        except Exception as e:
            print(f"the error is : {e}")
            
    def Ximg2Xbase(self)->np.ndarray:
        """
        Convert image coordinates to robot base coordinates using the transformation matrix and intrinsic matrix.

        Args:
            Ximg (list): Image coordinates [Ximg_x, Ximg_y, Ximg_depth/1000].
            robot_ip (str): The IP address of the robot.

        Returns:
            np.ndarray: The converted coordinates in the robot base frame as [Xbase_x, Xbase_y, Xbase_z].
        """
        try:
            T_cam_base = self.get_T_cam_base()
            Z = self.poses1[2]
            Ximg = np.array([self.poses1[0], self.poses1[1], 1.0])
            Xcam = Z * np.linalg.inv(self.inst_matrix) @ Ximg
            Xbase = T_cam_base @ np.array([Xcam[0], Xcam[1], Xcam[2], 1.0])        
            return Xbase[:3]
        except Exception as e:
            print(f"the error of Ximg2Xbase : {e}")
    
    def _run(self):
        try:
            while not rospy.is_shutdown():
                self.get_ximg(15)
                T_cam_base = self.get_T_cam_base()
                # Xbase =self.Ximg2Xbase()
                # self.oreintation = self.get_orientation(self.oreintation)
                # Xbase[1] = Xbase[1] + 0.4
                # Xbase[2] = Xbase[2] - 0.06
                # Xbase[0] = Xbase[0] + 0.3
                # Xbase[1] = Xbase[1] + 0.090
                # Xbase[2] = Xbase[2] - 0.100
                
                # t = TransformStamped()
                # t.header.stamp = rospy.Time.now()
                # t.header.frame_id ='base_link'
                # t.child_frame_id = 'aruco_marker_cam'
            
                # # Store the translation (i.e. position) information
                # t.transform.translation.x = Xbase[0]
                # t.transform.translation.y = Xbase[1]
                # t.transform.translation.z = Xbase[2]
                # rotation_matrix = np.eye(4)
                # rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(self.oreintation))[0]
                # r = R.from_dcm(rotation_matrix[0:3, 0:3])
                # quat = r.as_quat()   
                # # Quaternion format     
                # t.transform.rotation.x = quat[0] 
                # t.transform.rotation.y = quat[1] 
                # t.transform.rotation.z = quat[2] 
                # t.transform.rotation.w = quat[3] 
                # self.tfbroadcaster.sendTransform(t)  
                # Xbase = Xbase*1000

                # self.oreintation = np.degrees(self.oreintation)
                # position = [Xbase[0],Xbase[1],Xbase[2],self.oreintation[0],self.oreintation[1],self.oreintation[2]]
                # print(f'position:{position}')
        except Exception as e:
            print(f"the error of run: {e}")


if __name__=="__main__":
    TF_trans()._run()



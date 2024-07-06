#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

def pose_callback(pose_msg):
    try:
        print("Listening.....")
        # Create a TransformListener
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        print("Got the transforms")
        # Wait for the transform from 'camera_link' to 'base_link' to be available
        transform = tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time(0), rospy.Duration(4.0))

        # Transform the pose from 'camera_link' frame to 'base_link' frame
        transformed_pose = do_transform_pose(pose_msg, transform)

        # Log the transformed pose
        rospy.loginfo("Transformed Pose in base_link frame: {}".format(transformed_pose))
    
    except Exception as e:
        rospy.logerr("Transform error: {}".format(e))

def main():
  try:
    rospy.init_node('pose_transformer_node', anonymous=True)

    # Subscribe to the PoseStamped topic (assuming the topic is 'camera_pose')
    rospy.Subscriber('camera_pose', PoseStamped(), pose_callback)

    rospy.loginfo("Pose transformer node started.")
    rospy.spin()
  except Exception as e:
    print(e)
if __name__ == '__main__':
    main()

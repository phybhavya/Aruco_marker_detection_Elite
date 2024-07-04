#!/usr/bin/python3
import rospy
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

def create_chain_from_urdf():
    """
    Create a KDL chain from a URDF file.
    """
    # Load the URDF file from the parameter server
    robot = URDF.from_parameter_server()
    (ok, tree) = treeFromUrdfModel(robot)
    if not ok:
        raise ValueError("Failed to parse URDF model into KDL tree")
    base_link = robot.get_root()
    end_link = "camera_mount"  # Replace with your robot's end effector link name
    return tree.getChain(base_link, end_link)

def euler_to_kdl_rotation(roll, pitch, yaw):
    """
    Convert Euler angles to a KDL rotation object.
    """
    return kdl.Rotation.RPY(roll, pitch, yaw)

def main():
    """
    Main function to solve the inverse kinematics.
    """
    rospy.init_node('ik_solver')

    # Create the KDL chain from the URDF
    try:
        chain = create_chain_from_urdf()
    except Exception as e:
        rospy.logerr("Failed to create KDL chain from URDF: {}".format(e))
        return

    # Define the target end-effector position and orientation (in radians)
    target_position = kdl.Vector(0.32687306176327394, -0.5876923434020969, 0.13998529168226827)
    target_orientation = euler_to_kdl_rotation(0.0007, -0.000988, 0.031701)

    # Define the target frame
    target_frame = kdl.Frame(target_orientation, target_position)

    # Solver setup
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)
    ik_solver_vel = kdl.ChainIkSolverVel_pinv(chain)
    ik_solver_pos = kdl.ChainIkSolverPos_NR(chain, fk_solver, ik_solver_vel)

    # Initial guess for the joint positions
    initial_joint_positions = kdl.JntArray(chain.getNrOfJoints())
    for i in range(chain.getNrOfJoints()):
        initial_joint_positions[i] = 0.0

    # Solve IK
    result_joint_positions = kdl.JntArray(chain.getNrOfJoints())
    ret = ik_solver_pos.CartToJnt(initial_joint_positions, target_frame, result_joint_positions)

    if ret >= 0:
        joint_angles = [result_joint_positions[i] for i in range(result_joint_positions.rows())]
        # joint_angles = np.degrees(joint_angles)
        rospy.loginfo("Joint Angles (in radians): {}".format(joint_angles))
    else:
        rospy.logerr("IK solver failed with return code: {}".format(ret))

if __name__ == "__main__":
    main()
#-71.57,-81.69,120.315,328.352,-50.454,263.924 values by aruco
#-124.08961776859506, -97.40134297520663, 141.64201732673268, -43.42168209876542, 235.29706790123447, 87.20640432098764 #by solver
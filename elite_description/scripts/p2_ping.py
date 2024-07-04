#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time
import json
import socket
class robot_visualisation():
    def __init__(self):
        rospy.init_node('ping')
        self.conSuc = None
        self.sock = None
        self.robot_ip = "192.168.1.200"
        self.joint_state_pub = rospy.Publisher('/joint_states',JointState, queue_size=10)
        self.conSuc, self.sock = self.connectETController(self.robot_ip)
        rospy.loginfo("Inside the init function")
    def sendCMD(self,sock, cmd, params=None, id=1):
        if not params:
            params = []
        else:
            params = json.dumps(params)
        sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params, id) + "\n"
        try:
            sock.sendall(bytes(sendStr, "utf-8"))
            ret = sock.recv(1024)
            jdata = json.loads(str(ret, "utf-8"))
            if "result" in jdata.keys():
                return (True, json.loads(jdata["result"]), jdata["id"])
            elif "error" in jdata.keys():
                return (False, jdata["error"], jdata["id"])
            else:
                return (False, None, None)
        except Exception as e:
            rospy.logerr(f"Failed to send command: {e}")
            return (False, None, None)
        
    def connectETController(self,ip, port=8055):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((ip, port))
            return (True, sock)
        except Exception as e:
            sock.close()
            rospy.logerr(f"Failed to connect to controller: {e}")
            return (False, sock)
        

    def getjointangles(self):
        #conSuc, sock = self.connectETController(robot_ip)

        # Function to get the master point from the user
        if self.conSuc:
            ret, result, id = self.sendCMD(self.sock, "get_joint_pos")
            #sock.close()
            # print(result)
            return result
#        return None
    def publish_joint_state(self):
        """Publish joint states to the /joint_states topic."""
        joint_values = self.getjointangles()
        joint_values = np.radians(joint_values)
        if joint_values is not None:
            rospy.loginfo("Received joint angles from the robot")
            joint_state = JointState()
            joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
            joint_state.position = joint_values
            joint_state.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(joint_state)
            rospy.loginfo(f"Published joint values: {joint_values}")
        else:
            rospy.logwarn("Failed to receive joint angles from the robot")

    def main(self):        
        rate = rospy.Rate(10000)
        rospy.loginfo("Inside the main function")
        while not rospy.is_shutdown():
            rospy.loginfo("The node has started")
            self.publish_joint_state()
            # rate.sleep()


if __name__ == "__main__":
    robot_visualisation().main()



    









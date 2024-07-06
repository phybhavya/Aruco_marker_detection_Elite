#!/usr/bin/env python3
import socket
import json
import numpy as np
point = [705.75446,42.123, 71.72129918, -21.71,-39.915,-20.458]
def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        print("connection successful")
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, sock)  # Return the socket even in case of failure
def disconnectETController(sock):
    if sock:
        sock.close()
        sock = None
    else:
        sock = None
def sendCMD(sock, cmd, params=None, id=1):
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
        return (False, None, None)
if __name__ == "__main__":
    robot_ip = "192.168.1.200"
    conSuc,sock=connectETController(robot_ip)
    P000 = [0, -90 , 90, -90, 90, 0]
    print("connected")
    #if servo is on turn it off
    suc, result , id=sendCMD(sock, "getServoStatus")

    print(result)
    # suc, result , id=sendCMD(sock,"set_servo_status",{"status":1})
    # print(result)
    ret,flange_pose, id = sendCMD(sock,'get_base_flange_pose',{'unit_type': 0})
    print('flange_pose =',flange_pose)
    suc,actual_tcp , id=sendCMD(sock,'get_actual_tcp',{'tool_num':2,'user_num' :1})
    print(actual_tcp)
    suc , result , id = sendCMD(sock, "get_joint_pos")
    print(result)
    # suc , IK , id=sendCMD(sock,'inverseKinematic',{'targetPose':result,'referencePos':P000})
    # print(IK)
    suc , IK , id=sendCMD(sock,'inverseKinematic',{'targetPose':point,'unit_type': 0})#,'referencePos':result,'unit_type': 0})
    print(IK)
    suc , result , id=sendCMD(sock,'moveByJoint',{'targetPos':IK, 'speed':5, 'acc':10, 'dec' :10})
    print(result)


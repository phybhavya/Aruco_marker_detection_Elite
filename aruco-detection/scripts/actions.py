#!/usr/bin/env python3
import socket
import json
import numpy as np
point = [0, -431.4946133985377, 253.6014452907613,146.8620484184182, -87.00809584831633, -56.47124346827282]
point1 = [-318.3263988458562, -841.6112221540133, 319.46256675612165, 97.386206443409, 0.6469681011647631, -164.45539145750615]
point2 = [-468.2969177033729, 752.9219574431047, 160.0959480227565, -1.7746250136995376, 0.08370233663338932, -2.973973299897099]
IK_g = [-255.68535821190872, -39.29713661790511, 63.979872650794185, -59.649,280.880,90.889]
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
#reverse,trial1:-272.44970342706466, -295.6863566258049, 234.44407172621533,0.06392602080114507, -0.09482422439381091, 1.8136757513101238
#reverse,trial2:-247.13950659759305, -157.2951426510995, 268.0793390450891,0.09274190791970548, -0.11237564954536376, 1.7984114166445462
#reverse,trial 3:-230.21370659227495, 2.0695813517203203, 257.5309633992323,0.0639203550199 8515, -0.09482227485698899, 1.8136724707904137            self.get_ximg(15)
#failed #trial4: 274.3724504963711, -11179.132099019493, 15319.263066114123,0.06393233089998267, -0.09482385267665937, 1.8136751538573512
#trail5:13.552051367971044, -418.7156792319276, 108.41793862505524,0.06392602080114507, -0.09482422439381091, 1.8136757513101238
#trial6:14.949655348767562, -329.14326561589587, 108.60809613360689,0.08318580136229119, -0.06655781718531265, 1.8170198020734423
#trial7:14.949655348767562, -329.14326561589587, 108.60809613360689,  0.08318580136229119, -0.06655781718531265, 1.8170198020734423
#trial8:14.949530302785265, -276.5345601228009, 150.54162113416945,106.14162302712833, -87.57766964181856, -15.511510027533852
#trial9: -15.748155211048532, -299.35253862499667, 101.98344177419037, 0.0386043514368764, -0.0398757461308586, 2.051761995363011
#trial10: -209.1885885580278, -351.09472756803103, 155.5280063174676,0.05727397349631461, -0.10312173773350589, 1.4012355953006652


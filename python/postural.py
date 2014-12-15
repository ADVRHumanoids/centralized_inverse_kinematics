#!/usr/bin/python

import yarp
import numpy as np
import rospy
import tf
import PyKDL as kdl
from pydrc import pydrc
import pYTask

print "posture"


yarp.Network.init()

port_posture = yarp.BufferedPortBottle()
port_posture.open("/desired/posture/position/ref:o")
yarp.Network.connect("/desired/posture/position/ref:o", "/bigman/centralized_inverse_kinematics/postural/set_ref:i")

left_arm_names = ["LShSag"]
left_arm_values = [-0.4]    

bottle_posture = port_posture.prepare()
bottle_posture.clear()
pYTask.position_joint_msg(left_arm_names , left_arm_values, bottle_posture)

port_posture.write()   
print "Move " + left_arm_names
    

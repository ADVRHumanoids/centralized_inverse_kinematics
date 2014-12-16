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

left_arm_names = ["LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2"]
left_arm_values = [0.33, 0.18, 0.0, -1.3, 0.0, 0.0, 0.0]

right_arm_names = ["RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"]
right_arm_values = [0.33, -0.18, 0.0, -1.3, 0.0, 0.0, 0.0]

left_leg_names = ["LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat"]
left_leg_values = [0.0,  0.0, -0.43, 0.86, -0.43, 0.0]

right_leg_names = ["RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat"]
right_leg_values = [0.0,  0.0, -0.43, 0.86,  -0.43, 0.0]

torso_names = ["WaistLat", "WaistSag", "WaistYaw"]
torso_values = [0.0, 0.0, 0.0]

final_name_list = left_arm_names + right_arm_names + left_leg_names + right_leg_names + torso_names
final_value_list = left_arm_values + right_arm_values + left_leg_values + right_leg_values + torso_values

bottle_posture = port_posture.prepare()
bottle_posture.clear()
pYTask.position_joint_msg(final_name_list , final_value_list, bottle_posture)

port_posture.write()
print "Moving: "
print final_name_list
print "in: "
print final_value_list
    

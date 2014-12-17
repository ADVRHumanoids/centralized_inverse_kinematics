#!/usr/bin/python

import yarp
import numpy as np
import rospy
import tf
import PyKDL as kdl
from pydrc import pydrc
import pYTask

print "Homing"

if __name__ == '__main__':
    rospy.init_node('Homing')

    yarp.Network.init()

    port_posture = yarp.BufferedPortBottle()
    port_posture.open("/desired/posture/position/ref:o")
    yarp.Network.connect("/desired/posture/position/ref:o", "/bigman/centralized_inverse_kinematics/postural/set_ref:i")

    port_Waist = yarp.BufferedPortBottle()
    port_Waist.open("/bigman/centralized_inverse_kinematics/cartesian::Waist/set_ref:o")
    yarp.Network.connect("/bigman/centralized_inverse_kinematics/cartesian::Waist/set_ref:o",
                         "/bigman/centralized_inverse_kinematics/cartesian::Waist/set_ref:i")

    listener = tf.TransformListener()

    timeoutDuration = rospy.Duration(secs=10)
    source_frame='world'
    target_frame='Waist'
    listener.waitForTransform(source_frame, target_frame, time=rospy.Time(0), timeout=timeoutDuration)

    success = False
    while not success:
        try:
            (world_pos_Waist, world_rot_Waist) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("ERROR on lookupTransform")
            continue

    Waist_pos_d = kdl.Frame()
    Waist_pos_d.p = kdl.Vector(world_pos_Waist[0], world_pos_Waist[1], 1.0)#1.05


    bottle_Waist = port_Waist.prepare()
    bottle_Waist.clear()
    pYTask.pose_msg(Waist_pos_d, source_frame, target_frame, bottle_Waist)
    port_Waist.write()

    left_arm_names = ["LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2"]
    left_arm_values = [0.33,     0.18,     0.0,     -1.3,     0.0,             0.0,     0.0]

    right_arm_names = ["RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"]
    right_arm_values = [0.33,    -0.18,    0.0,      -1.3,     0.0,             0.0,     0.0]

    #torso_names = ["WaistLat", "WaistSag", "WaistYaw"]
    #torso_values = [0.0, 0.0, 0.0]

    final_name_list = left_arm_names + right_arm_names #+ torso_name
    final_value_list = left_arm_values + right_arm_values #+ torso_values

    bottle_posture = port_posture.prepare()
    bottle_posture.clear()
    pYTask.position_joint_msg(final_name_list , final_value_list, bottle_posture)

    port_posture.write()


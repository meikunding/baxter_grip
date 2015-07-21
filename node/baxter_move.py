#!/usr/bin/env python

import argparse
import struct
import sys
import numpy as np
import rospy
import baxter_interface
import math

from math import fabs

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String,Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

quaternion_x =0.748
quaternion_y =-0.664
quaternion_z =0.023
quaternion_w =-0.006

last_position=[0,0,0]

TEST_JOINT_ANGLES = dict()
TEST_JOINT_ANGLES['left'] = dict()
TEST_JOINT_ANGLES['left']['left_s0'] = -0.3857
TEST_JOINT_ANGLES['left']['left_s1'] = -0.9085
TEST_JOINT_ANGLES['left']['left_e0'] =  0.0529
TEST_JOINT_ANGLES['left']['left_e1'] = 1.9140 
TEST_JOINT_ANGLES['left']['left_w0'] = -0.09165
TEST_JOINT_ANGLES['left']['left_w1'] = 0.58291
TEST_JOINT_ANGLES['left']['left_w2'] = 0.20823

#Accepts PoseStamped() message and moves towards it

def BaxterMovement(new_position):


    limb = "left"
    ns = "ExternalTools/" + limbw + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    cur_pointx = new_position.x
    cur_pointy = new_position.y
    cur_pointz = new_position.z
    dif_x =fabs(cur_pointx-last_position[0])
    dif_y =fabs(cur_pointy-last_position[1])
    dif_z =fabs(cur_pointz-last_position[2])
    last_position[0] = cur_pointx
    last_position[1] = cur_pointy
    last_position[2] = cur_pointz
    print dif_x
    if (dif_x>=20 or dif_y>=20 or dif_z >= 20):
        print "target coordinate change\n"
        print cur_pointx/1000
        print cur_pointy/1000
        print cur_pointz/1000+0.2
        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
        move_to_pose.pose.position=Point(
                            x=cur_pointx/1000-0.1,
                            y=cur_pointy/1000,
                            z=cur_pointz/1000+0.2,
                            )
        move_to_pose.pose.orientation=Quaternion(
                            x=0.748,
                            y=-0.664,
                            z=0.023,
                            w=0.006,
                            )
        #Request IK Service Client with new pose to move to
        ikreq.pose_stamp.append(move_to_pose)
        try:
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            return 1
        print resp

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)

        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                        }.get(resp_seeds[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                          (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "baxter will move"
            limb = baxter_interface.Limb('left')
            limb.move_to_joint_positions(limb_joints)
            #rospy.sleep(5)
            #left.set_joint_positions(joint_angles['left'])
        else:
            rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
    else:
        print "target do not move"


    #Move left limb to limb_joints found from IK service


    return 0




#Main section of code to run

def main():

    global last_position
    rospy.init_node('baxtermovement',anonymous = True)

    #Enables Baxter
    rospy.loginfo("ENTERED THE MOVEMENT LOOP")
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rospy.loginfo("Enabling robot... ")
    rs.enable()
    #left = baxter_interface.Limb('left')
    #joint_angles = dict()
    #joint_angles['left'] = dict()
    #joint_angles = TEST_JOINT_ANGLES
    #print joint_angles
    #left.set_joint_positions(joint_angles['left'])
    #Calibrate and open left gripper
    #baxterleft = baxter_interface.Gripper('left')
    #baxterleft.calibrate()
    #baxterleft.open()


    #Subscribe to topic for PoseStamped messages to be sent to
    rospy.Subscriber("/position",Point,BaxterMovement)


    rospy.spin()



if __name__ == '__main__':
    main()

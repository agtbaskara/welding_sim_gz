#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CompressedImage
from welding_sim_gz.srv import MoveRobot, MoveRobotResponse
import moveit_commander

imoving = False

# def ismoving_cb(msg):
#     global ismoving
#     ismoving = msg.data

# def handle_move_robot(req):
#     global ismoving
#     rospy.loginfo("Moving Robot")
#     target_pose_pub.publish(req.targetpose)
#     ismoving = True
#     while ismoving:
#         pass
#     rospy.loginfo("Movement Done")
#     image = rospy.wait_for_message("/camera", Image, timeout=None)
#     time.sleep(1)
#     return MoveRobotResponse(image)

if __name__ == "__main__":
    rospy.init_node("simulator_bridge")

    # #define subscriber
    # ismoving_sub = rospy.Subscriber("/ismoving", Bool, ismoving_cb)
    # #define publisher
    # target_pose_pub = rospy.Publisher("/targetpose", Pose, queue_size=1)
    # #define service
    # move_robot_srv = rospy.Service("/move_robot", MoveRobot, handle_move_robot)

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # Joint Target
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -math.pi/4
    # joint_goal[2] = 0
    # joint_goal[3] = -math.pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = math.pi/3
    # joint_goal[6] = 0

    # group.go(joint_goal, wait=True)
    # group.stop()
    # group.clear_pose_targets()

    # Pose Target
    pose_goal = Pose()
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7071068
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.7071068
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.8

    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()

    # WP1
    print("WP1")
    pose_goal.position.y = 0.4
    pose_goal.position.z = 0.9
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP2
    print("WP2")
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.9
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP3
    print("WP3")
    pose_goal.position.y = -0.4
    pose_goal.position.z = 0.9
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP4
    print("WP4")
    pose_goal.position.y = -0.4
    pose_goal.position.z = 0.6
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP5
    print("WP5")
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.6
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP6
    print("WP6")
    pose_goal.position.y = 0.4
    pose_goal.position.z = 0.6
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP7
    print("WP7")
    pose_goal.position.y = 0.4
    pose_goal.position.z = 0.2
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP8
    print("WP8")
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.2
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    # WP9
    print("WP9")
    pose_goal.position.y = -0.4
    pose_goal.position.z = 0.2
    group.go(pose_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    print("WP Done, Taking Picture!")
    time.sleep(1)

    print("DONE")

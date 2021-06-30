#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
from welding_sim_gz.srv import MoveRobotPose, MoveRobotPoseResponse, MoveRobotJoint, MoveRobotJointResponse
import moveit_commander

class MoveGroupInterface():
    def __init__(self):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.image = CompressedImage()
        
    def update_image(self, msg):
        self.image = msg
    
    def handle_move_robot_pose(self, req):
        rospy.loginfo("Moving Robot")
        self.group.go(req.targetpose, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        rospy.loginfo("Movement Done")
        return MoveRobotPoseResponse(self.image)
    
    def handle_move_robot_joint(self, req):
        rospy.loginfo("Moving Robot")
        self.group.go(req.targetjoint, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        rospy.loginfo("Movement Done")
        return MoveRobotJointResponse(self.image)

if __name__ == "__main__":
    rospy.init_node("simulation_bridge")

    arm_robot = MoveGroupInterface()

    # Define Subscriber
    rospy.Subscriber("/rrbot/camera1/image_raw/compressed", CompressedImage, arm_robot.update_image)

    # Define Service
    move_robot_pose_srv = rospy.Service("/simulation_bridge/move_robot_pose", MoveRobotPose, arm_robot.handle_move_robot_pose)
    move_robot_joint_srv = rospy.Service("/simulation_bridge/move_robot_joint", MoveRobotJoint, arm_robot.handle_move_robot_joint)

    rospy.spin()

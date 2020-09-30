#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##
from time import sleep
import time 
import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from math import pi
from std_msgs.msg import String , Float32MultiArray
from moveit_commander.conversions import pose_to_list   

## END_SUB_TUTORIAL

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("robot")

def callback (data):

    print data.data

    xq,yq,zq,wq = eul2quat(data.data[3],data.data[4],data.data[5])

    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.position.x = data.data[0]
    pose_goal.position.y = data.data[1]
    pose_goal.position.z = data.data[2]
    pose_goal.orientation.x = xq
    pose_goal.orientation.y = yq
    pose_goal.orientation.z = zq
    pose_goal.orientation.w = wq
    print pose_goal

    group.set_goal_orientation_tolerance(1.5)
    group.set_goal_position_tolerance(0.1)
    group.set_pose_target(pose_goal)
    plan = group.plan()
    group.execute(plan,wait=True)
    #sleep(10)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    #pose_goal.position.x = 0.48
    #pose_goal.position.y = 0.08
    #pose_goal.position.z = 0.73
    #pose_goal.orientation.x = 0.62
    #pose_goal.orientation.y = -0.48   
    #pose_goal.orientation.z = 0.615
    #pose_goal.orientation.w = 0.066
def eul2quat(roll,pitch,yaw):
    eul=[    roll,    pitch   , yaw]
    c = [math.cos(eul[0]/2),math.cos(eul[1]/2),math.cos(eul[2]/2)]
    s = [math.sin(eul[0]/2),math.sin(eul[1]/2),math.sin(eul[2]/2)]
    q = [c[ 1-1]*c[ 2-1]*c[ 3-1] - s[ 1-1]*s[ 2-1]*s[ 3-1], s[ 1-1]*c[ 2-1]*c[ 3-1] + c[ 1-1]*s[ 2-1]*s[ 3-1],-s[ 1-1]*c[ 2-1]*s[ 3-1] + c[ 1-1]*s[ 2-1]*c[ 3-1],c[ 1-1]*c[ 2-1]*s[ 3-1] + s[ 1-1]*s[ 2-1]*c[ 3-1]]
    return q
  
if __name__ == '__main__':

    print "Program start listerning..."
    rospy.init_node('callback', anonymous=True)
    data = rospy.Subscriber("/cam_object_pose", Float32MultiArray, callback, queue_size =10)      
    rospy.spin()
    #rate = rospy.Rate(10)  # 10hz
    print "============ Python group demo complete!"

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL

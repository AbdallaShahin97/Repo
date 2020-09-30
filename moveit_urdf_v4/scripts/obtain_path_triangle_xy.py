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
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list   
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    #self.sub = rospy.Subscriber('/unity_ros', JointState, self.sub_callback)
    #self.data = JointState()   
    #print(self.data)

 ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "robot"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("BiEST")

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self,J):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = J[0]
    joint_goal[1] = J[1]
    joint_goal[2] = J[2]
    joint_goal[3] = J[3]
    joint_goal[4] = J[4]
    joint_goal[5] = J[5]
    joint_goal[6] = J[6]
#    3  [0.9044557  2.0983732  1.2160792  0.17579198 0.9545673  0.43416858]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal , wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()



    ## END_SUB_TUTORIAL
    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose().pose

    #return all_close(joint_goal, current_joints, 0.01)
    return current_pose

  def get_current_pose_value(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    joint_goal = move_group.get_current_joint_values()

#    3  [0.9044557  2.0983732  1.2160792  0.17579198 0.9545673  0.43416858]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal , wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()



    ## END_SUB_TUTORIAL
    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.move_group.get_current_joint_values()
    current_pose = self.move_group.get_current_pose()

    #return all_close(joint_goal, current_joints, 0.01)
    return current_pose,current_joints


  def go_to_pose_goal(self,P):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:



    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.position.x = P[0]+0.000001
    pose_goal.position.y = P[1]+0.000001
    pose_goal.position.z = P[2]+0.000001

    print (pose_goal)
    #pose_goal.position.x = 0.48
    #pose_goal.position.y = 0.08
    #pose_goal.position.z = 0.73
    #pose_goal.orientation.x = 0.62
    #pose_goal.orientation.y = -0.48
    #pose_goal.orientation.z = 0.615
    #pose_goal.orientation.w = 0.066

    #rint(pose_goal)
    move_group.set_goal_orientation_tolerance(2)
    move_group.set_goal_position_tolerance(0.0001)

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    
    #plan = move_group.plan()
    #move_group.execute(plan,wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose

    #return all_close(pose_goal, current_pose, 0.1)
    return current_pose

  
def main():
  try:
    #print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    #from IK_DNN_py2 import IK_DNN
    from export_csv import export_csv_pose_moveit,export_csv_pose_DNN
    from IK_DNN_py2 import IK_DNN

    import math
    import numpy as np
    #print "============ Press `Enter` to execute a movement using a joint state goal ..."
    #raw_input()
    home_position=[0,0,0,0,0,0,0]
    #position_1=[0.351732,0.9044557,2.0983732,1.2160792,0.17579198,0.9545673,0.43416858]
    #from IK_DNN import *
    tutorial.go_to_joint_state(home_position)
    time.sleep(2)

    
    #np.savetxt('pose.out', [xp,yp,zp,x,y,z,w], delimiter=',')
    #file1 = open("state_joints.out","r+")  
    sz=0.75
    
    sx1=0
    sy1=0
    
    sx2=0.1
    sy2=0
    
    sx3=0.05
    sy3=0.1
    
    
    
    no_points=21
    co=0
    Ps1=[]
    Ps2=[]
    poses_moveit=[]

    true_xyz=[]
    import numpy as np
    import math


    tx12=np.linspace(sx1,sx2,no_points/3)
    ty12=np.linspace(sy1,sy2,no_points/3)

    tx23=np.linspace(sx2,sx3,no_points/3)
    ty23=np.linspace(sy2,sy3,no_points/3)

    tx31=np.linspace(sx3,sx1,no_points/3)
    ty31=np.linspace(sy3,sy1,no_points/3) 

    true_xyz=[]

    for i in range(no_points/3):
        xp=tx12[i]
        yp=ty12[i]
        zp=sz
        true_xyz.append([xp,yp,zp])
    for i in range(no_points/3):
        xp=tx23[i]
        yp=ty23[i]
        zp=sz
        true_xyz.append([xp,yp,zp])
    for i in range(no_points/3):
        xp=tx31[i]
        yp=ty31[i]
        zp=sz
        true_xyz.append([xp,yp,zp])
    print (true_xyz)
    

    
    for i in range(len(true_xyz)):
        current_pose=tutorial.go_to_pose_goal(true_xyz[i])
        xp=current_pose.position.x
        yp=current_pose.position.y
        zp=current_pose.position.z
        xo=current_pose.orientation.x
        yo=current_pose.orientation.y
        zo=current_pose.orientation.z
        wo=current_pose.orientation.w
        poses_moveit.insert(i,[xp,yp,zp,xo,yo,zo,wo])
        time.sleep(1)
    export_csv_pose_moveit(poses_moveit,"a_triangle_true.csv")

    print(poses_moveit)
    tutorial.go_to_joint_state(home_position)
    print ("$$$$$ Open the CAMERA $$$$$")
    time.sleep(5)
    tutorial.go_to_joint_state([0,0,0,0,0,0,0])
    # old_current_joint_states=home_position
    old_current_joint_states = [0,0,0,0,0,0,0]
    time.sleep(1)

    poses_DNN=[]
    for i in range(len(poses_moveit)):
        no_iterations=3
        factor_changes_joints=0
        weights_changes_joints=np.array([5,2,2,2,1,1,1])
        desired_pose=np.array([true_xyz[i][0],true_xyz[i][1],true_xyz[i][2],poses_moveit[i][3],poses_moveit[i][4],poses_moveit[i][5],poses_moveit[i][6]])
        print(desired_pose)
        predicted_joint_states=IK_DNN(desired_pose,old_current_joint_states,no_iterations,factor_changes_joints,weights_changes_joints)
        old_current_joint_states=predicted_joint_states
        current_pose=tutorial.go_to_joint_state(predicted_joint_states)
        time.sleep(1)
        xp=current_pose.position.x
        yp=current_pose.position.y
        zp=current_pose.position.z
        xo=current_pose.orientation.x
        yo=current_pose.orientation.y
        zo=current_pose.orientation.z
        wo=current_pose.orientation.w
        poses_DNN.insert(i,[xp,yp,zp,xo,yo,zo,wo])

    export_csv_pose_DNN(poses_DNN,"a_triangle_DNN.csv")




    print "============ Python tutorial demo complete!!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

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

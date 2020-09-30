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
from Models_py2 import *
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
    
    #move_group.set_planner_id("RRTstarConfigDefault")

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
    current_pose = self.move_group.get_current_pose()

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


  def go_to_pose_goal(self):
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
    pose_goal = geometry_msgs.msg.PoseStamped()
    
    pose_goal.pose.position.x = 0.484259
    pose_goal.pose.position.y = 0.081616
    pose_goal.pose.position.z = 0.732889
    pose_goal.pose.orientation.x = 0.620281
    pose_goal.pose.orientation.y =-0.482155
    pose_goal.pose.orientation.z = 0.615088
    pose_goal.pose.orientation.w = 0.066669

    #pose_goal.position.x = 0.48
    #pose_goal.position.y = 0.08
    #pose_goal.position.z = 0.73
    #pose_goal.orientation.x = 0.62
    #pose_goal.orientation.y = -0.48
    #pose_goal.orientation.z = 0.615
    #pose_goal.orientation.w = 0.066

    print(pose_goal)

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    #plan = move_group.go(wait=True)
    plan = move_group.plan()
    move_group.execute(plan,wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose()

    return all_close(pose_goal, current_pose, 0.1)


  
def main():
  try:
    #print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    from IK_DNN_py2 import IK_DNN

    #print "============ Press `Enter` to execute a movement using a joint state goal ..."
    #raw_input()
    home_position=[0,0,0,0,0,0,0]
    #position_1=[0.351732,0.9044557,2.0983732,1.2160792,0.17579198,0.9545673,0.43416858]
    #from IK_DNN import *
  
    import numpy as np
    import csv
    import time
    #np.savetxt('pose.out', [xp,yp,zp,x,y,z,w], delimiter=',')
    #file1 = open("state_joints.out","r+")  
    tutorial.go_to_joint_state(home_position)
    time.sleep(2)
    old_current_pose_value,old_current_joint_states=tutorial.get_current_pose_value()
    xp_old=old_current_pose_value.pose.position.x
    yp_old=old_current_pose_value.pose.position.y
    zp_old=old_current_pose_value.pose.position.z
    xo_old=old_current_pose_value.pose.orientation.x
    yo_old=old_current_pose_value.pose.orientation.y
    zo_old=old_current_pose_value.pose.orientation.z
    wo_old=old_current_pose_value.pose.orientation.w
    old_current_pose=[xp_old,yp_old,zp_old,xo_old,yo_old,zo_old,wo_old]
    continue_value='y'
    while continue_value=='y':
        current_pose_value,current_joint_states=tutorial.get_current_pose_value()
        xp=current_pose_value.pose.position.x
        yp=current_pose_value.pose.position.y
        zp=current_pose_value.pose.position.z
        xo=current_pose_value.pose.orientation.x
        yo=current_pose_value.pose.orientation.y
        zo=current_pose_value.pose.orientation.z
        wo=current_pose_value.pose.orientation.w
        desired_pose=[xp,yp,zp,xo,yo,zo,wo]
        #print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
        #print (old_current_pose_value)
        #print(current_pose_value)
        #print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
        if old_current_pose==desired_pose:
            pass
            
        else:
        #print(current_pose_value)
        
            xp=current_pose_value.pose.position.x
            yp=current_pose_value.pose.position.y
            zp=current_pose_value.pose.position.z
            xo=current_pose_value.pose.orientation.x
            yo=current_pose_value.pose.orientation.y
            zo=current_pose_value.pose.orientation.z
            wo=current_pose_value.pose.orientation.w
            desired_pose=[xp,yp,zp,xo,yo,zo,wo]
            #desired_pose=[x_pos,y_pos,z_pos,x_ori,y_ori,z_ori,w_ori]
            #desired_pose=[-0.364,-0.168,0.485,0.241,0.673,-0.623,-0.314]
            no_iterations=5
            factor_changes_joints=0.1
            weights_changes_joints=np.array([5,2,2,2,1,1,1])
            desired_pose=[0.5515146255493164, 0.03615140914916992, 0.7865891456604004, 0.005129251629114151, -0.9897263050079346, -0.04617518186569214, 0.1352166384458542]

            predicted_joint_states=IK_DNN(desired_pose,old_current_joint_states,no_iterations,factor_changes_joints,weights_changes_joints)
            #position_1=[0.351732,0.9044557,2.0983732,1.2160792,0.17579198,0.9545673,0.43416858]
            #from IK_DNN import *
          
            #np.savetxt('pose.out', [xp,yp,zp,x,y,z,w], delimiter=',')
            #file1 = open("state_joints.out","r+")  
          

            #position_1= (file1.read() )
            print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
            #print(position_1)
            print('Old joint states')
            print(current_joint_states)


            #pp=[0 ,  1.012388517, 3.752457892, 2.00712864 , 1.570792654 ,1.68421585 , 2.967063402]   
             #position_1=IK_DNN([xp,yp,zp,x,y,z,w])

            states=tutorial.go_to_joint_state(predicted_joint_states)
            xp=states.pose.position.x
            yp=states.pose.position.y
            zp=states.pose.position.z
            xo=states.pose.orientation.x
            yo=states.pose.orientation.y
            zo=states.pose.orientation.z
            wo=states.pose.orientation.w

            #time.sleep(3)
            #tutorial.go_to_joint_state(home_position)
            print('***************************************************')
            print('The desired pose:')
            print(desired_pose)
            print('The achieved pose:')
            print([round(xp,6),round(yp,6),round(zp,6),round(xo,6),round(yo,6),round(zo,6),round(wo,6)])
            print('***************************************************')
            d1=np.array([round(xp,6),round(yp,6),round(zp,6),round(xo,6),round(yo,6),round(zo,6),round(wo,6)])
            d2=np.array(desired_pose)
            d3=abs(d1-d2)
            print('The absolute error:')
            print([round(d3[0]*1000,3),round(d3[1]*1000,3),round(d3[2]*1000,3),round(d3[3],3),round(d3[4],3),round(d3[5],3),round(d3[6],3)])
            print('The mean absolute error:')
            print(sum(d3)/6)
            old_current_pose_value,old_current_joint_states=tutorial.get_current_pose_value()
            xp_old=old_current_pose_value.pose.position.x
            yp_old=old_current_pose_value.pose.position.y
            zp_old=old_current_pose_value.pose.position.z
            xo_old=old_current_pose_value.pose.orientation.x
            yo_old=old_current_pose_value.pose.orientation.y
            zo_old=old_current_pose_value.pose.orientation.z
            wo_old=old_current_pose_value.pose.orientation.w
            old_current_pose=[xp_old,yp_old,zp_old,xo_old,yo_old,zo_old,wo_old]
            print('Current joint states')
            print(old_current_joint_states)
            #print(states)
            #print "============ Press `Enter` to execute a movement using a pose goal ..."
            #raw_input()
            #tutorial.go_to_pose_goal()

        continue_value=raw_input('Do you want to execute this pose? [y/n]')


    print "============ Python tutorial demo complete!"
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

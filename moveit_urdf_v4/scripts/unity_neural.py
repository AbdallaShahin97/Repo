#!/usr/bin/env python
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped
from Models_py2 import *



moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("robot")
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

oldvalue=0
posearray = []

def callback(data):
  print("Starting Here")
  newvalue=data.pose
  global oldvalue
  global posy
  if newvalue != oldvalue:
    pass
    print "GOTEEE"
    posy=newvalue
    print posy
    group.set_planner_id("BiEST")
    
    group.set_goal_orientation_tolerance(1)
    group.set_goal_position_tolerance(0.05)
    #group.allow_looking(True)
    group.set_pose_target(posy)
    #plan1 = group.plan()
    # print "============ Visualizing plan"
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()      
    # display_trajectory.trajectory.append(plan1)
    # display_trajectory_publisher.publish(display_trajectory)
    #group.execute(plan1)
    #group.clear_pose_targets()
    group.go(posy, wait=True)  #
    # Calling `stop()` ensures that there is no residual movement
    group.stop()

    oldvalue=newvalue

    print "Neural Starting"
  
    from IK_DNN_py2 import IK_DNN
    import numpy as np
    import csv
    import time

    #current_pose_value=group.get_current_pose()
    xp=posy.position.x
    yp=posy.position.y
    zp=posy.position.z
    xo=posy.orientation.x
    yo=posy.orientation.y
    zo=posy.orientation.z
    wo=posy.orientation.w
    desired_pose=[xp,yp,zp,xo,yo,zo,wo]
    #print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
    #print (old_current_pose_value)
    #print(current_pose_value)
    #print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')

    #desired_pose=[x_pos,y_pos,z_pos,x_ori,y_ori,z_ori,w_ori]
    #desired_pose=[-0.364,-0.168,0.485,0.241,0.673,-0.623,-0.314]
    no_iterations=3
    factor_changes_joints=0.1
    weights_changes_joints=np.array([5,2,2,2,1,1,1])

    predicted_joint_states=group.get_current_joint_values()
    predicted_joint_states=IK_DNN(desired_pose,[0,0,0,0,0,0,0],no_iterations,factor_changes_joints,weights_changes_joints)
    #position_1=[0.351732,0.9044557,2.0983732,1.2160792,0.17579198,0.9545673,0.43416858]
    #from IK_DNN import *
  
    #np.savetxt('pose.out', [xp,yp,zp,x,y,z,w], delimiter=',')
    #file1 = open("state_joints.out","r+")  
  

    #position_1= (file1.read() )
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    #print(position_1)
    #print('Old joint states')
    #print(current_joint_states)


    #pp=[0 ,  1.012388517, 3.752457892, 2.00712864 , 1.570792654 ,1.68421585 , 2.967063402]   
     #position_1=IK_DNN([xp,yp,zp,x,y,z,w])


    group.go(predicted_joint_states, wait=True)
    group.stop()

    states=group.get_current_pose()

    xp=states.pose.position.x
    yp=states.pose.position.y
    zp=states.pose.position.z
    xo=states.pose.orientation.x
    yo=states.pose.orientation.y
    zo=states.pose.orientation.z
    wo=states.pose.orientation.w

    #time.sleep(3)
    #group.go_to_joint_state(home_position)
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




if __name__ == '__main__':
    print "Program start listerning..."
    rospy.init_node('callback', anonymous=True)
    data = rospy.Subscriber("/odom", PoseStamped, callback, queue_size =1)      
    rospy.spin()
    
    #pose_target =0
    #print "============ Press `Enter` to begin the group by setting up the moveit_commander (press ctrl-d to exit) ..."
    #raw_input()

    print "============ Python group demo complete!"



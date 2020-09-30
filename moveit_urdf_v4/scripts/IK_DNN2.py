from Models import *


def IK_DNN(J):
  import math
  from math import atan2
  import numpy
  import numpy as np
  ### The desired pose with respect to F0 ####
  ### Position ###
  ### The desired pose with respect to F0 ####
  ### Position ###
  xp=J[0]
  yp=J[1]
  zp=J[2]
  ### Orientation "Quatranion" ###
  x=J[3]
  y=J[4]
  z=J[5]
  w=J[6]

  import math
  from math import atan2
  import numpy
  import numpy as np

  ### The desired pose with respect to F0 ####
  ### Position ###
  #xp=0.484199
  #yp=0.081584
  #zp=0.732913
  ### Orientation "Quatranion" ###
  #x=0.6202
  #y=-0.4822
  #z=0.6151
  #w=0.0666
  #eh el kalam ba2a ya zizo
  

  dp=[xp,yp,zp,x,y,z,w]
  ######## Current Joint States ##########
  current_joints=numpy.array([ 0 , 0 , 0 , 0 , 0 , 0 , 0 ])
  ############## Preferences ##############
  weights_changes_joints=numpy.array([5,2,2,2,1,1,1])
  factor_changes_joints=0.1
  no_possibilities_j1=2
  ###########################################################33
  from math import atan2
  def quat2eul(qx ,qy , qz, qw):
    aSinInput = 2*(qx*qz + qy*qw)
    if aSinInput>1:
      aSinInput=1
    if aSinInput<-1:
      aSinInput=-1
    eul = [ math.atan2( -2*(qy*qz - qx*qw), qw**2 - qx**2 - qy**2 + qz**2 ), math.asin( aSinInput ), math.atan2( -2*(qx*qy - qz*qw), qw**2 + qx**2 - qy**2 - qz**2 )]
    return eul
  def eul2quat(roll,pitch,yaw):
    eul=[    roll,    pitch   , yaw]
    c = [math.cos(eul[0]/2),math.cos(eul[1]/2),math.cos(eul[2]/2)]
    s = [math.sin(eul[0]/2),math.sin(eul[1]/2),math.sin(eul[2]/2)]
    q = [c[ 1-1]*c[ 2-1]*c[ 3-1] - s[ 1-1]*s[ 2-1]*s[ 3-1], s[ 1-1]*c[ 2-1]*c[ 3-1] + c[ 1-1]*s[ 2-1]*s[ 3-1],-s[ 1-1]*c[ 2-1]*s[ 3-1] + c[ 1-1]*s[ 2-1]*c[ 3-1],c[ 1-1]*c[ 2-1]*s[ 3-1] + s[ 1-1]*s[ 2-1]*c[ 3-1]]
    return q
  eul=quat2eul(x,y,z,w)
  desired_pose_wrt_f0=[xp , yp , zp   ,  eul[2] ,   eul[1]   , eul[0]]
  poses=[]
  for i in range(no_possibilities_j1):
    x=desired_pose_wrt_f0[0]-0.150
    y=desired_pose_wrt_f0[1]-0.0975-i*(0.7914/(no_possibilities_j1-1))
    z=desired_pose_wrt_f0[2]
    desired_pose_wrt_f2=[x,y,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
    poses.insert(i,desired_pose_wrt_f2)
  print("Transformed poses generation: Done")
  import numpy
  import numpy as np
  import math
  def predicted_joint_states_rectifier (sol):
    j2=sol[0]
    j3=sol[1]
    j4=sol[2]
    j5=sol[3]
    j6=sol[4]
    j7=sol[5]
    off_range=0

    if j2<-1.0471  :
      j2=-1.0471
    elif j2>1.0471:
      j2=1.047

    if j3<0 :
      j3=0
    elif j3 >3.8222:
      j3=3.8222

    if j4<0 :
      j4=0
    elif  j4>2.0769:
      j4=2.0769
    
    if j5<-1.5708 :
      j5=-1.5708
    elif  j5>1.5708:
      j5=1.5708

    if j6<0 :
      j6=0
    elif  j6>1.7802:
      j6=1.7802
      
    if j7<-3.1416:
      j6=-3.1416
    elif  j7>3.1416:
      j7=3.1416

    if j6>=0 and j6<=1.7802:
      n6=math.ceil(4*j6/1.7802)
    else:
      n6=0
    if j7>=-3.1416 and j7<-1.5708:
      n7=1
    elif j7>=-1.5708 and j7<0:
        n7=2
    elif j7>=0 and j7<=3.1416:
      n7=math.ceil(2*j7/3.1416)+2
    else:
      n7=0
    if off_range==1:
      n6=0
      n7=0
    sol[0]=j2
    sol[1]=j3
    sol[2]=j4
    sol[3]=j5
    sol[4]=j6
    sol[5]=j7
    return sol,n6,n7
  def predicted_pose_from_forward(sol_old):
    t=1.1
    sol,n6,n7=predicted_joint_states_rectifier(sol_old)
    #print(sol)
    num1 = str(n6) 
    num2 = str(n7) 
    num3 = num1+num2 
    n6_n7=int(num3)
    #n6_n7= int(f"{n6}{n7}")
    forward = {
    11: model11_forward,12: model12_forward, 13: model13_forward,14: model14_forward,
    21: model21_forward,22: model22_forward,23: model23_forward,24: model24_forward,
    31: model31_forward,32: model32_forward,33: model33_forward,34: model34_forward,
    41: model41_forward,42: model42_forward,43: model43_forward,44: model44_forward,}
    if n6_n7 in forward.keys():
     x=forward[n6_n7].predict([[np.float(sol[0]),np.float(sol[1]),np.float(sol[2]),np.float(sol[3]),np.float(sol[4]),np.float(sol[5])]])
    else:
      x=[np.array([99999 ,99999 ,99999, 99999 ,99999 ,99999])]
      n6_n7=99
    return x[0] , n6_n7,sol;
  ################################################################################################
  inverse = {
    0: [model11_inverse,11],1: [model12_inverse,12],2: [model13_inverse,13],3: [model14_inverse,14],
    4: [model21_inverse,21],5: [model22_inverse,22],6: [model23_inverse,23],7: [model24_inverse,24],
    8: [model31_inverse,31],9: [model32_inverse,32],10: [model33_inverse,33],11: [model34_inverse,34],
    12: [model41_inverse,41],13: [model42_inverse,42],14: [model43_inverse,43],15: [model44_inverse,44],}
  inverse2 = {
    11: [model11_inverse,11],12: [model12_inverse,11],13: [model13_inverse,11],14: [model14_inverse,11],
    21: [model21_inverse,11],22: [model22_inverse,11],23: [model23_inverse,11],24: [model24_inverse,11],
    31: [model31_inverse,11],32: [model32_inverse,11],33: [model33_inverse,11],34: [model34_inverse,11],
    41: [model41_inverse,11], 42: [model42_inverse,11], 43: [model43_inverse,11],44: [model44_inverse,11],}
  outputs={}
  for n in range(16):
    for i in range(no_possibilities_j1):
      sol=inverse[n][0].predict([poses[i]])[0]
      predicted_j1=i*(0.7914/(no_possibilities_j1-1))
      predicted_pose,no_forward,sol_rectified=predicted_pose_from_forward(sol)

      predicted_joints=numpy.array([predicted_j1, sol_rectified[0],sol_rectified[1],sol_rectified[2],sol_rectified[3],sol_rectified[4],sol_rectified[5]])
      changes_in_joints=abs(predicted_joints - current_joints)

      error_changes_joints = (sum(changes_in_joints*weights_changes_joints)/sum(weights_changes_joints))/7
      #print(predicted_pose)
      error_pose=sum(abs(poses[i]-predicted_pose))/6
      error=factor_changes_joints*error_changes_joints+(1-factor_changes_joints)*error_pose
      #no_inverse= int(f"{inverse[n][1]}{i}")
      p1=str(inverse[n][1])
      p2=str(i)
      p3=p1+p2
      no_inverse=int(p3)
      outputs[no_inverse] = [error,inverse[n][1],no_forward,i,error_changes_joints,error_pose]
  m=min(outputs.items(), key=lambda x: x[1]) 
  print(m)
  print("################ Here is the best solution ###############")
  print('Total error:')
  print(m[1][0])
  print('Error in changes in joints states:')
  print(m[1][4])
  print('Error in changes in predicted pose:')
  print(m[1][5])
  print('which inverse neural network used:')
  print(m[1][1])
  print('which forward neural network used:')
  print(m[1][2])
  print('Number of j1 possibilities entered by the user:')
  print(no_possibilities_j1)
  print ('So the desired pose to reach is:')
  #p0=poses[m[1][3]]
  #p2=[p0[0]+0.15,p0[1]+0.0975+m[1][3]*(0.7914/(no_possibilities_j1-1)),p0[2],p0[3],p0[4],p0[5]]
  #print(p2)
  print(dp)
  print("And the expected pose to take place is:")  
  A,B,C=predicted_pose_from_forward(inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0])
  r=4
  #D=[round(A[0]+0.15,r),round(A[1]+0.0975+m[1][3]*(0.7914/(no_possibilities_j1-1)),r),round(A[2],r),round(A[3],r),round(A[4],r),round(A[5],r)]
  q=eul2quat(A[5],A[4],A[3])
  E=[round(A[0]+0.15,r),round(A[1]+0.0975+m[1][3]*(0.7914/(no_possibilities_j1-1)),r),round(A[2],r),round(q[1],r),round(q[2],r),round(q[3],r),round(q[0],r)]

  print(E)    
  print('And the best solution is:')
  s_old=inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0]
  s_new,k1,k2=predicted_joint_states_rectifier(s_old)
  print([m[1][3]*(0.7914/(no_possibilities_j1-1)),round(s_new[0],r),round(s_new[1],r),round(s_new[2],r),round(s_new[3],r),round(s_new[4],r),round(s_new[5],r)])

  print('Thank you!')

  #-0.0975-i*(0.7914/(no_possibilities_j1-1))

  return([m[1][3]*(0.7914/(no_possibilities_j1-1)),round(s_new[0],r),round(s_new[1],r),round(s_new[2],r),round(s_new[3],r),round(s_new[4],r),round(s_new[5],r)])

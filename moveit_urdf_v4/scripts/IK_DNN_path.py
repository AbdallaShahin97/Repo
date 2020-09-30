from Models_py2 import *

import math
def IK_DNN(P,J,no_iterations,factor_changes_joints,weights_changes_joints):
  import math
  from math import atan2
  import numpy
  import numpy as np
  ### The desired pose with respect to F0 ####
  ### Position ###
  ### The desired pose with respect to F0 ####
  ### Position ###
  xp=P[0]
  yp=P[1]
  zp=P[2]
  ### Orientation "Quatranion" ###
  x=P[3]
  y=P[4]
  z=P[5]
  w=P[6]
  j1_current=J[0]
  j2_current=J[1]
  j3_current=J[2]
  j4_current=J[3]
  j5_current=J[4]
  j6_current=J[5]
  j7_current=J[6]
  dp=[xp,yp,zp,x,y,z,w]
  ######## Current Joint States ##########
  current_joints=numpy.array([ j1_current , j2_current , j3_current , j4_current , j5_current , j6_current , j7_current ])
  ############## Preferences ##############
  #weights_changes_joints=numpy.array([5,2,2,2,1,1,1])
  #factor_changes_joints=0.3
  #no_iterations=3
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
  min_J1=0
  max_J1=0.7914
  mid_J1=(min_J1+max_J1)/2

  x=desired_pose_wrt_f0[0]-0.150
  y_min=desired_pose_wrt_f0[1]-0.0975-min_J1
  y_max=desired_pose_wrt_f0[1]-0.0975-max_J1
  y_mid=desired_pose_wrt_f0[1]-0.0975-mid_J1
  z=desired_pose_wrt_f0[2]
  desired_pose_wrt_f2_min=[x,y_min,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
  desired_pose_wrt_f2_max=[x,y_max,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
  desired_pose_wrt_f2_mid=[x,y_mid,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]

  poses.insert(0,desired_pose_wrt_f2_min)
  poses.insert(1,desired_pose_wrt_f2_mid)
  poses.insert(2,desired_pose_wrt_f2_max)

  #print(poses)
  #print("Transformed poses generation: Done")
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
    n6=0
    n7=0
    if j2<=-1.0471  :
      j2=-1.0471
    elif j2>=1.0471:
      j2=1.047

    if j3<=0 :
      j3=0
    elif j3 >=3.8222:
      j3=3.8222

    if j4<=0 :
      j4=0
    elif  j4>=2.0769:
      j4=2.0769
    
    if j5<=-1.5708 :
      j5=-1.5708
    elif  j5>=1.5708:
      j5=1.5708
    if j6<=0 :
      j6=0
    elif  j6>=1.7802:
      j6=1.7802
      
    if j7<=-3.1416:
      j7=-3.1416
    elif  j7>=3.1416:
      j7=3.1416
    if j6>0 and j6<=1.7802:
      n6=int(math.ceil(4*j6/1.7802))
    elif j6==0:
      n6=1
    if j7>=-3.1416 and j7<-1.5708:
      n7=1
    elif j7>=-1.5708 and j7<0:
        n7=2
    elif j7>=0 and j7<=3.1416:
      n7=int(math.ceil(2*j7/3.1416)+2)

    sol[0]=j2
    sol[1]=j3
    sol[2]=j4
    sol[3]=j5
    sol[4]=j6
    sol[5]=j7

   # print('%%%%%')
    #print(n6)
    #print(n7)
    
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
  J1_value={0:[min_J1],1:[mid_J1],2:[max_J1]}
  #print(J1_value[1][0])
  #print('******************************************')
  f=0
  m=0
  outputs={}

  for c in range(no_iterations):
    print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    outputs={}

    for i in range(3):

      for n in range(16):
        f=f+1
        sol=inverse[n][0].predict([poses[i]])[0]
        predicted_j1=J1_value[i][0]
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
        #print(predicted_j1)
        #print(outputs[no_inverse])
    #print(outputs)
    m=min(outputs.items(), key=lambda x: x[1]) 
    #print(m)
    best=m[1][3]
    #print(best)

    if best==0:
      min_J1=min_J1
      max_J1=mid_J1
      mid_J1=(min_J1+max_J1)/2
    elif best==1:
      min_J1=(min_J1+mid_J1)/2
      max_J1=(max_J1+mid_J1)/2
      mid_J1=(min_J1+max_J1)/2
    elif best==2:
      min_J1=mid_J1
      max_J1=max_J1
      mid_J1=(min_J1+max_J1)/2
    J1_value={0:[min_J1],1:[mid_J1],2:[max_J1]}
    
    poses=[]
    x=desired_pose_wrt_f0[0]-0.150
    y_min=desired_pose_wrt_f0[1]-0.0975-min_J1
    y_max=desired_pose_wrt_f0[1]-0.0975-max_J1
    y_mid=desired_pose_wrt_f0[1]-0.0975-mid_J1
    z=desired_pose_wrt_f0[2]
    desired_pose_wrt_f2_min=[x,y_min,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
    desired_pose_wrt_f2_max=[x,y_max,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
    desired_pose_wrt_f2_mid=[x,y_mid,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]

    poses.insert(0,desired_pose_wrt_f2_min)
    poses.insert(1,desired_pose_wrt_f2_mid)
    poses.insert(2,desired_pose_wrt_f2_max)

  m=min(outputs.items(), key=lambda x: x[1]) 
  best=m[1][3]
  #print (f)
  print("################ Here is the best solution ###############")
  print('Total error:')
  print(m[1][0])
  print('Error in changes in joints states:')
  print(m[1][4])
  print('Error in changes in predicted pose:')
  print(m[1][5])
  print('Contribution factor of changes in joints entered by the user: ')
  print(factor_changes_joints)
  print('which inverse neural network used:')
  print(m[1][1])
  print('which forward neural network used:')
  print(m[1][2])
  print('Number of j1 grid search iterations entered by the user:')
  print(no_iterations)
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
  E=[round(A[0]+0.15,r),round(A[1]+0.0975+J1_value[best][0],r),round(A[2],r),round(q[1],r),round(q[2],r),round(q[3],r),round(q[0],r)]

  print(E)    
  print('And the best solution is:')
  s_old=inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0]
  s_new,k1,k2=predicted_joint_states_rectifier(s_old)
  print([J1_value[best][0],round(s_new[0],r),round(s_new[1],r),round(s_new[2],r),round(s_new[3],r),round(s_new[4],r),round(s_new[5],r)])

  print('Thank you!')

  return([J1_value[best][0],round(s_new[0],r),round(s_new[1],r),round(s_new[2],r),round(s_new[3],r),round(s_new[4],r),round(s_new[5],r)])

def frange(start, stop=None, step=None):

    if stop == None:
        stop = start + 0.0
        start = 0.0

    if step == None:
        step = 1.0

    while True:
        if step > 0 and start >= stop:
            break
        elif step < 0 and start <= stop:
            break
        yield ("%g" % start) # return float number
        start = start + step
[0.7, 0.466667, 0.5557189779913265, 0.81, 0, 0.58, 1]

sx=0.7
sy=0.5
sz=0.65
r=0.1
no_points=20
co=0
Ps=[]


for i in frange(sy-r,sy+r+2*r/((no_points/2)-1),2*r/((no_points/2)-1)):
  a=1
  b=-2*sz
  i=float(i)
  c=-(r**2-i**2+2*i*sy-sy**2-sz**2)
  zz1=(-b-math.sqrt(b**2-4*a*c))/2*a
  Ps.insert(co,[sx,i,zz1,0.81,0,0.58,1])
  co=co+1
  zz2=(-b+math.sqrt(b**2-4*a*c))/2*a
  Ps.insert(co,[sx,i,zz2,0.81,0,0.58,1])
  co=co+1
  
print(Ps)
J=[0,0,0,0,0,0,0]
no_iterations=3
factor_changes_joints=0
weights_changes_joints=[5,2,2,2,1,1,1]
solutions=[]
for i in range(len(Ps)):
  J= IK_DNN(Ps[i],J,no_iterations,factor_changes_joints,weights_changes_joints)
  solutions.insert(i,J)
  print(J)
  pass
print(solutions)


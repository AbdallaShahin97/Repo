#from Models import *
from Models_v2 import *

def IK_DNN(J):
  ########################################################################################################
  import math
  from math import atan2
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
  for i in range(10):
    x=desired_pose_wrt_f0[0]-0.150
    y=desired_pose_wrt_f0[1]-0.0975-i*0.087933
    z=desired_pose_wrt_f0[2]
    desired_pose_wrt_f2=[x,y,z,desired_pose_wrt_f0[3],desired_pose_wrt_f0[4],desired_pose_wrt_f0[5]]
    poses.insert(i,desired_pose_wrt_f2)
  print("Transformed poses generation: Done")

  #################################################################################################
  import numpy
  import numpy as np
  import math
  def predicted_pose_from_forward(sol):
    j6=sol[4]
    j7=sol[5]
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
    return x[0] , n6_n7;
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
    for i in range(10):
      sol=inverse[n][0].predict([poses[i]])[0]
      predicted_pose,no_forward=predicted_pose_from_forward(sol)
      error=sum(abs(poses[i]-predicted_pose))/6
      p1=str(inverse[n][1])
      p2=str(i)
      p3=p1+p2
      no_inverse=int(p3)


      #no_inverse= int(f"{inverse[n][1]}{i}")
      outputs[no_inverse] = [error,inverse[n][1],no_forward,i]
  m=min(outputs.items(), key=lambda x: x[1]) 
  print("#########################################3")
  print('least error')
  print(m[1][0])
  print('which inverse')
  print(m[1][1])
  print('which forward')
  print(m[1][2])
  print('value of j1')
  print(m[1][3]*0.087933)
  print ('So the pose is:')
  print (poses[m[1][3]])
  print('and the best solution is:')
  P=inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0]
  print(inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0])
  print('done')
  return([m[1][3]*0.087933,P[0],P[1],P[2],P[3],P[4],P[5]])
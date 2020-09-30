# load and evaluate a saved model
#from google.colab import drive
#drive.mount('/content/drive')
import pandas as pd
import tensorflow as tf
from array import array
from tensorflow import keras
from tensorflow.keras import backend as K
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Dense
from sklearn import preprocessing
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import MinMaxScaler
from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers import *
from tensorflow.python.keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import MinMaxScaler
from scipy import io
from keras.models import load_model
from keras import backend as K
def custom_activation(x):
    return (K.tanh(x) * 2.718**(0.2*x))    
custom_obj = {}
custom_obj['custom_activation'] = custom_activation
####### Forward kinematics models #########
model11_forward = tf.keras.models.load_model("Forward_whole_pose_11.h5",custom_objects = custom_obj)
model12_forward = tf.keras.models.load_model("Forward_whole_pose_12.h5",custom_objects = custom_obj)
model13_forward = tf.keras.models.load_model("Forward_whole_pose_13.h5",custom_objects = custom_obj)
model14_forward = tf.keras.models.load_model("Forward_whole_pose_14.h5",custom_objects = custom_obj)

model21_forward = tf.keras.models.load_model("Forward_whole_pose_21.h5",custom_objects = custom_obj)
model22_forward = tf.keras.models.load_model("Forward_whole_pose_22.h5",custom_objects = custom_obj)
model23_forward = tf.keras.models.load_model("Forward_whole_pose_23.h5",custom_objects = custom_obj)
model24_forward = tf.keras.models.load_model("Forward_whole_pose_24.h5",custom_objects = custom_obj)

model31_forward = tf.keras.models.load_model("Forward_whole_pose_31.h5",custom_objects = custom_obj)
model32_forward = tf.keras.models.load_model("Forward_whole_pose_32.h5",custom_objects = custom_obj)
model33_forward = tf.keras.models.load_model("Forward_whole_pose_33.h5",custom_objects = custom_obj)
model34_forward = tf.keras.models.load_model("Forward_whole_pose_34.h5",custom_objects = custom_obj)

model41_forward = tf.keras.models.load_model("Forward_whole_pose_41.h5",custom_objects = custom_obj)
model42_forward = tf.keras.models.load_model("Forward_whole_pose_42.h5",custom_objects = custom_obj)
model43_forward = tf.keras.models.load_model("Forward_whole_pose_43.h5",custom_objects = custom_obj)
model44_forward = tf.keras.models.load_model("Forward_whole_pose_44.h5",custom_objects = custom_obj)

####### Inverse kinematics models #########
model11_inverse = tf.keras.models.load_model("Inverse_whole_pose_11.h5",custom_objects = custom_obj)
model12_inverse = tf.keras.models.load_model("Inverse_whole_pose_12.h5",custom_objects = custom_obj)
model13_inverse = tf.keras.models.load_model("Inverse_whole_pose_13.h5",custom_objects = custom_obj)
model14_inverse = tf.keras.models.load_model("Inverse_whole_pose_14.h5",custom_objects = custom_obj)

model21_inverse = tf.keras.models.load_model("Inverse_whole_pose_21.h5",custom_objects = custom_obj)
model22_inverse = tf.keras.models.load_model("Inverse_whole_pose_22.h5",custom_objects = custom_obj)
model23_inverse = tf.keras.models.load_model("Inverse_whole_pose_23.h5",custom_objects = custom_obj)
model24_inverse = tf.keras.models.load_model("Inverse_whole_pose_24.h5",custom_objects = custom_obj)

model31_inverse = tf.keras.models.load_model("Inverse_whole_pose_31.h5",custom_objects = custom_obj)
model32_inverse = tf.keras.models.load_model("Inverse_whole_pose_32.h5",custom_objects = custom_obj)
model33_inverse = tf.keras.models.load_model("Inverse_whole_pose_33.h5",custom_objects = custom_obj)
model34_inverse = tf.keras.models.load_model("Inverse_whole_pose_34.h5",custom_objects = custom_obj)

model41_inverse = tf.keras.models.load_model("Inverse_whole_pose_41.h5",custom_objects = custom_obj)
model42_inverse = tf.keras.models.load_model("Inverse_whole_pose_42.h5",custom_objects = custom_obj)
model43_inverse = tf.keras.models.load_model("Inverse_whole_pose_43.h5",custom_objects = custom_obj)
model44_inverse = tf.keras.models.load_model("Inverse_whole_pose_44.h5",custom_objects = custom_obj)

print("Uploading models: Done")

########################################################################################################
import math
from math import atan2
### The desired pose with respect to F0 ####
### Position ###

#file1 = open("pose.out","r+")  
  

#pose_d= (file1.read() )
xp=0.484199
yp=0.081584
zp=0.732913
        ### Orientation "Quatranion" ###
x=0.6202
y=-0.4822
z=0.6151
w=0.0666

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
print(inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0])
p=round(inverse2[m[1][1]][0].predict([poses[m[1][3]]])[0],4)
print('done')
np.savetxt('state_joints.out', [round(m[1][3]*0.087933,4),p[0],p[1],p[2],p[3],p[4],p[5]], delimiter=',')
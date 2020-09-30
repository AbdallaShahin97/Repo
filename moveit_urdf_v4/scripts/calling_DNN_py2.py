#from Models import *
#from IK_DNN3 import IK_DNN
#import csv


import csv
#pp=[   -0.3618  , 0.5064 ,   0.6713 ,  -0.5733 ,   0.3657  ,  0.7288   ,-0.0804]
#xp=pp[0]+0.15
#yp=pp[1]+0.097
#zp=pp[2]
#x=pp[3]
#y=pp[4]
#z=pp[5]
#w=pp[6]
xp=-0.364
yp=-0.168
zp=0.485
        ### Orientation "Quatranion" ###
x=0.241
y=0.673
z=-0.623
w=-0.314

f2 = open('desired_pose.csv', 'w')

with f2:

    writer = csv.writer(f2)
    
    #for row in nms:
    writer.writerow([xp,yp,zp,x,y,z,w])


#f2 = open('output1.csv', 'r')

#with f2:

  #  reader = csv.reader(f2)
    
  #  for row in reader:
        
   #     print(row)
#predicted_joint_states=[0,0,0,0,0,0,0]
#print(type(row[0]))
#for i in range(7):
#	predicted_joint_states[i]=float(row[i])
#print('#########3')
#print(predicted_joint_states)
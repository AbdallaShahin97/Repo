
xp=0.484199
yp=0.081584
zp=0.732913
        ### Orientation "Quatranion" ###
x=0.6202
y=-0.4822
z=0.6151
w=0.0666


#position_1=IK_DNN([xp,yp,zp,x,y,z,w])
continue_entering='y'
while continue_entering=='y':
	desired_pose = list(map(float,input("\nEnter the numbers : ").strip().split()))[:7] 
	print (desired_pose)

	print('*********')
	position_1=IK_DNN(desired_pose)

	f = open('output1.csv', 'w')

	with f:

	    writer = csv.writer(f)
	    
	    #for row in nms:
	    writer.writerow(position_1)

	print(position_1)
	print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')

	f2 = open('output1.csv', 'r')

	with f2:

	    reader = csv.reader(f2)
	    
	    for row in reader:
	        
	        print(row)
	predicted_joint_states=[0,0,0,0,0,0,0]
	print(type(row[0]))
	for i in range(7):
		predicted_joint_states[i]=float(row[i])
	print('#########3')
	print(predicted_joint_states)
	continue_entering=input('Do you want to continue y or n ?')
print('Thank you, see you soon!')
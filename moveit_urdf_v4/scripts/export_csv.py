import csv
def export_csv_pose_DNN(poses,name):

	f2 = open(name, 'w')

	with f2:

		writer = csv.writer(f2)
		    
		    #for row in nms:
		for i in range(len(poses)):

			writer.writerow(poses[i])
def export_csv_pose_moveit(poses,name):

	f2 = open(name, 'w')

	with f2:

		writer = csv.writer(f2)
		    
		    #for row in nms:
		for i in range(len(poses)):

			writer.writerow(poses[i])

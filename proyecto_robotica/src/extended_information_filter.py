#! /usr/bin/env python3

from queue import Empty
import rospy
from tf import TransformBroadcaster
from std_msgs.msg import Empty
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import *
import rosgraph
import rostopic
from rospy import Time
import time
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


beacon_positions = {"beacon1": [ 10.0, 10.0,  10.0],
					"beacon2": [  2.0,  2.0,  0.0],
					"beacon3": [ 10.0,-10.0,  5.0],
					"beacon4": [-10.0,-10.0,  10.0],
					"beacon5": [-10.0, 10.0,  5.0]}

beacon_msgs = {}

drone_ground_truth_pose = PoseStamped()


T = 1
beacon_std_dev = 0.5
varModelo = 1
ValorOmega = 0.01

# Initial conditions
IC = np.array( [[0],  # x
				[0],  # y
				[0],  # z
				[0],  # x-1
				[0],  # y-1
				[0],  # z-1
				[0],  # vx
				[0],  # vy
				[0]]) # vz

# G Matrix (Our model A)
G = np.array([[  1,   0,   0,   0,   0,   0, T, 0, 0],
	 [  0,   1,   0,   0,   0,   0, 0, T, 0],
	 [  0,   0,   1,   0,   0,   0, 0, 0, T],
	 [  1,   0,   0,   0,   0,   0, 0, 0, 0],
	 [  0,   1,   0,   0,   0,   0, 0, 0, 0],
	 [  0,   0,   1,   0,   0,   0, 0, 0, 0],
	 [1/T,   0,   0,-1/T,   0,   0, 0, 0, 0],
	 [  0, 1/T,   0,   0,-1/T,   0, 0, 0, 0],
	 [  0,   0, 1/T,   0,   0,-1/T, 0, 0, 0]])


# R Matrix (How much we trust the model of our drone dynamics)
R = np.diag([   varModelo,
				varModelo, 
				varModelo, 
				0.00001, 
				0.00001, 
				0.00001, 
				varModelo*0.1, 
				varModelo*0.1, 
				varModelo*0.1])
  
# Information matrix (Inverse of the covariance matrix)
Omegak = np.diag([ValorOmega,
				  ValorOmega,
				  ValorOmega,
				  ValorOmega*10,
				  ValorOmega*10,
				  ValorOmega*10,
				  ValorOmega*0.1,
				  ValorOmega*0.1,
				  ValorOmega*0.1])

# Information vector
Xik = Omegak @ IC

def extended_information_filter():

	# Get number of beacons
	dim = len(beacon_msgs)
 
	# Build Q matrix (How much we trust the measurements)
	Q = np.diag([beacon_std_dev**2]*dim)
	
	global Omegak, Xik
	
	Omegak_inv = inv(Omegak)
	Q_inv = inv(Q)
 
	### PREDICTION ###

	# Mean of the previous prediction step
	Mu = Omegak_inv @ Xik  
	# Information matrix of the previous prediction step (Inv(Covariance matrix at time k))
	Omegak_1_p = inv(G @ Omegak_inv @ G.T + R)

	gk = G @ Mu # Next time step in our predition
	Mu_1_p = gk	# Mean of the next prediction step
	Xik_1_p = Omegak_1_p @ gk # Information vector of the next prediction step
 
	### UPDATE ###
	
	Ht = np.zeros((1,9), dtype=float) # Its always dimention 9 by number of beacons 
	ht = np.zeros((1,1), dtype=float) # ht has only 1 colum	
	z  = np.zeros((1,1), dtype=float) # z has only 1 colum too

	# Calculate jacobian Ht and ht
	for beacon_name in beacon_msgs:
		# Beacon measurement
		dist_beacon2robot_x = beacon_msgs[beacon_name].pose.position.x
		dist_beacon2robot_y = beacon_msgs[beacon_name].pose.position.y
		dist_beacon2robot_z = beacon_msgs[beacon_name].pose.position.z
  
		# Beacon position
		beacon_pos_x = beacon_positions[beacon_name][0]
		beacon_pos_y = beacon_positions[beacon_name][1]
		beacon_pos_z = beacon_positions[beacon_name][2]

		# Distance from beacon to robot
		dist_beacon2robot_norm = np.linalg.norm([dist_beacon2robot_x,dist_beacon2robot_y,dist_beacon2robot_z])
  
		# Jacobian or sensor sensitivity matrix
		norm = np.linalg.norm([(beacon_pos_x - Mu_1_p[0][0]), (beacon_pos_y - Mu_1_p[1][0]), (beacon_pos_z - Mu_1_p[2][0])])
	
		Ht = np.vstack([Ht,[-(beacon_pos_x - Mu_1_p[0][0])/norm, -(beacon_pos_y - Mu_1_p[1][0])/norm, -(beacon_pos_z - Mu_1_p[2][0])/norm, 0, 0, 0, 0, 0, 0]])
		# Prediction of the measurment
		ht = np.vstack([ht,norm])
		# Beacon measurement vector
		z  = np.vstack([z, dist_beacon2robot_norm]) 

	Ht = Ht[1:]
	ht = ht[1:]
	z  = z[1:] 
 
	Omegak_1 = Omegak_1_p + Ht.T @ Q_inv @ Ht
	Xik_1 = Xik_1_p + Ht.T @ Q_inv @ (z - ht + Ht @ Mu_1_p)
	
	# Convert my information vector to our moment space
	Omegak_1_inv = inv(Omegak_1)
	Mu_1 = Omegak_1_inv@Xik_1
	
	# Update information variables for next iteration
	Omegak = Omegak_1
	Xik = Xik_1
 
	return  Mu_1

def beacon_callback(msg):
	global beacon_msgs
	beacon_msgs[msg.header.frame_id] = msg

def localization_publisher():

	# FILTER  	
	Xk = extended_information_filter()
 
	print("Xk: ", Xk)
 	
	drone_pose = PoseStamped()
	drone_pose.header.frame_id = "drone_frame"
	drone_pose.header.stamp = Time.now()
	drone_pose.pose.position.x = Xk[0][0]
	drone_pose.pose.position.y = Xk[1][0]
	drone_pose.pose.position.z = Xk[2][0]
	drone_pose.pose.orientation.x = 0
	drone_pose.pose.orientation.y = 0
	drone_pose.pose.orientation.z = 0
	drone_pose.pose.orientation.w = 1
	
  # Publish the beacon location with respect to world frame
	tf_broadcaster = TransformBroadcaster()
	translation = (Xk[0][0],Xk[1][0],Xk[2][0])
	rotation = (0, 0, 0, 1)
	tf_broadcaster.sendTransform(translation, rotation, Time.now(), "drone_frame", "world")
 
	# Publish results
	global drone_ground_truth_pose	
	msg = Float64MultiArray()
	mydataDim = MultiArrayDimension()
	mydataDim.size = 9
	mydataDim.label = "results"
	mydataDim.stride = 1
	msg.layout.dim.append(mydataDim)
	msg.data = [Xk[0], Xk[1], Xk[2], drone_ground_truth_pose.pose.position.x, drone_ground_truth_pose.pose.position.y, drone_ground_truth_pose.pose.position.z]
	results_publ.publish(msg)

def ground_truth_callback(msg):
    global drone_ground_truth_pose
    drone_ground_truth_pose = msg

rospy.init_node('EIF_node')
rate = rospy.Rate(1)


master = rosgraph.Master('/rostopic')
pubs, subs = rostopic.get_topic_list(master=master)
beacon_sub = {}

time.sleep(5)

for topic in pubs:
	if topic[0].startswith('/beacon'):
		beacon_sub[topic[0]] = rospy.Subscriber(topic[0], PoseStamped, beacon_callback)
		beacon_msgs[topic[0][1:]] = PoseStamped() # Initialize the beacon msg for each beacon
		print(topic[0])
  

ground_truth = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, ground_truth_callback)
results_publ = rospy.Publisher('/results', Float64MultiArray, queue_size=1)


while not rospy.is_shutdown():
	localization_publisher()
	rate.sleep()

rospy.spin()    
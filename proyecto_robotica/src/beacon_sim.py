#! /usr/bin/env python3

import rospy
import rospkg
import os
import tf
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel, GetModelState
from std_msgs.msg import Empty
import tf2_ros

beacon_positions = {"beacon1": [ 10.0, 10.0,  10.0],
					"beacon2": [  2.0,  2.0,  0.0],
					"beacon3": [ 10.0,-10.0,  5.0],
					"beacon4": [-10.0,-10.0,  10.0],
					"beacon5": [-10.0, 10.0,  5.0]}

drone_ground_truth_pose = PoseStamped()

# Beacon noise parameters
beacon_std_dev = 0.5
beacon_normal = 0


def spawn_sdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        get_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        state = get_state(name,'')

        if state.success == False:
            spawn_sdf(name, description_xml, "/", pose, reference_frame)

    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e)) 


def ground_truth_callback(msg):
    global drone_ground_truth_pose
    drone_ground_truth_pose = msg


def beacon_publisher():

    for beacon_name in beacon_positions:

        distance = PoseStamped()

        noise = np.random.normal(beacon_normal, beacon_std_dev, 3)

        # Calculate distance between drone and beacon
        distance.pose.position.x = drone_ground_truth_pose.pose.position.x - beacon_positions[beacon_name][0]
        distance.pose.position.y = drone_ground_truth_pose.pose.position.y - beacon_positions[beacon_name][1]
        distance.pose.position.z = drone_ground_truth_pose.pose.position.z - beacon_positions[beacon_name][2]

        # Add noise to the measurement
        distance.pose.position.x = distance.pose.position.x + noise[0]
        distance.pose.position.y = distance.pose.position.y + noise[1]
        distance.pose.position.z = distance.pose.position.z + noise[2]
        
        # Publish the distance along with name of beacon
        distance.header.frame_id = beacon_name
        distance.header.stamp = rospy.Time.now()
        beacon_pub[beacon_name].publish(distance)


        # Publish the becaon location with respect to world frame
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf_msg_1 = geometry_msgs.msg.TransformStamped()
        tf_msg_1.header.stamp = rospy.Time.now()
        tf_msg_1.header.frame_id = "world"
        tf_msg_1.child_frame_id = beacon_name 
        tf_msg_1.transform.translation.x = beacon_positions[beacon_name][0]
        tf_msg_1.transform.translation.y = beacon_positions[beacon_name][1]
        tf_msg_1.transform.translation.z = beacon_positions[beacon_name][2]

        tf_msg_1.transform.rotation.x = 0
        tf_msg_1.transform.rotation.y = 0
        tf_msg_1.transform.rotation.z = 0
        tf_msg_1.transform.rotation.w = 1

        broadcaster.sendTransform(tf_msg_1)
        
        
        # Publish the becaon location with respect to world frame
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf_msg_2 = geometry_msgs.msg.TransformStamped()
        tf_msg_2.header.stamp = rospy.Time.now()
        tf_msg_2.header.frame_id = "world"
        tf_msg_2.child_frame_id = beacon_name + "_measurement"
        tf_msg_2.transform.translation.x = beacon_positions[beacon_name][0] +distance.pose.position.x
        tf_msg_2.transform.translation.y = beacon_positions[beacon_name][1] +distance.pose.position.y
        tf_msg_2.transform.translation.z = beacon_positions[beacon_name][2] +distance.pose.position.z

        tf_msg_2.transform.rotation.x = 0
        tf_msg_2.transform.rotation.y = 0
        tf_msg_2.transform.rotation.z = 0
        tf_msg_2.transform.rotation.w = 1

        broadcaster.sendTransform(tf_msg_2)
        
    

rospy.init_node('beacon_sim')
rate = rospy.Rate(1)


beacon_pub = {}
for beacon_name in beacon_positions:
    beacon_pub[beacon_name] = rospy.Publisher(beacon_name, PoseStamped, queue_size=1)

ground_truth = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, ground_truth_callback)

listener = tf.TransformListener()

product_xml = ""

rospack = rospkg.RosPack()
package_dir = rospack.get_path('proyecto_robotica')
model_directory = package_dir + '/models/beacon.sdf'


for beacon_name in beacon_positions:
    beacon_pose = Pose()
    beacon_pose.position.x = beacon_positions[beacon_name][0]
    beacon_pose.position.y = beacon_positions[beacon_name][1]
    beacon_pose.position.z = beacon_positions[beacon_name][2]

    product_xml = open(model_directory, "r").read()
    product_xml = product_xml.replace("beacon", beacon_name)

    spawn_sdf(beacon_name, product_xml, beacon_pose, "world")

while not rospy.is_shutdown():
    beacon_publisher()
    rate.sleep()

empty_msg = Empty()
twist_msg = Twist()

rospy.spin()    
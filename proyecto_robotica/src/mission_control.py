#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def go(x,y,z,r,p,yaw,time):
  twist_msg.linear.x = x
  twist_msg.linear.y = y
  twist_msg.linear.z = z
  twist_msg.angular.x = r
  twist_msg.angular.y = p
  twist_msg.angular.z = yaw
  i = 0
  while i < time:
      cmd_vel_pub.publish(twist_msg)
      i = i + 1
      rate.sleep()

def takeoff_callback(msg): 
  # Drone takeoff
  go(0,0,2.4,0,0,0,3)
  # Stop
  go(0,0,0,0,0,0,1)
  
def land_callback(msg): 
  # Drone land
  go(0,0,-2.4,0,0,0,30)
  # Stop
  go(0,0,0,0,0,0,1)

def start_flight_callback(msg): 
  
  # Drone takeoff
  go(0,0,2.4,0,0,0,3)
  
  # Trayectoria cuadrada
  go(1,0,0,0,0,0,7)
  go(0,1,0,0,0,0,7)
  go(-1,0,0,0,0,0,14)
  go(0,-1,0,0,0,0,14)
  go(1,0,0,0,0,0,14)
  go(0,1,0,0,0,0,7)
  
  # Espiral
  go(0,2.45,0.5,0,0,0.35,30)
  # Stop
  go(0,0,0,0,0,0,1)
  

rospy.init_node('mission_control')
rate = rospy.Rate(1)

takeoff_sub = rospy.Subscriber('/takeoff', Empty, takeoff_callback)
land_sub = rospy.Subscriber('/land', Empty, land_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
start_flight = rospy.Subscriber('/start_flight', Empty, start_flight_callback)

empty_msg = Empty()
twist_msg = Twist()

rospy.spin()


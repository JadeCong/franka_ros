#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import numpy as np
import time


def franka_state_publisher_gazebo_node():
    # init ros node
    rospy.init_node("franka_state_publisher_gazebo", anonymous=True)
    
    # get ros parameters from parameter server
    gazebo_service_type = rospy.get_param("/franka_state_publisher_gazebo/gazebo_service_type")
    franka_object = rospy.get_param("/franka_state_publisher_gazebo/franka_object")
    franka_state_topic = rospy.get_param("/franka_state_publisher_gazebo/franka_state_topic")
    
    # define the publisher and msgs for franka state
    pub_franka_state = rospy.Publisher(franka_state_topic, PoseStamped, queue_size=1, tcp_nodelay=True, latch=False)
    franka_ee_pose = PoseStamped()
    
    # define the service proxy for getting franka_ee_pose
    srv_get_franka_state = rospy.ServiceProxy(gazebo_service_type, GetLinkState)
    req_franka_state = GetLinkStateRequest()
    req_franka_state.link_name = franka_object
    time.sleep(2)  # wait 2 seconds for gazebo getting started
    
    # get franka_ee_pose from gazebo service and publish the msgs
    while True:
        # call the service to get franka state in gazebo
        franka_state = srv_get_franka_state(req_franka_state)
        
        # set the franka_ee_pose
        franka_ee_pose.header.stamp = rospy.Time.now()
        franka_ee_pose.header.frame_id = franka_state.link_state.link_name
        franka_ee_pose.pose.position = franka_state.link_state.pose.position
        franka_ee_pose.pose.orientation = franka_state.link_state.pose.orientation
        
        # publish the franka_ee_pose
        pub_franka_state.publish(franka_ee_pose)


if __name__ == "__main__":
    # run ros node
    franka_state_publisher_gazebo_node()

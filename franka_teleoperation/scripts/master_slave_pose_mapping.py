#!/usr/bin/env python

import sys, os, time
from subprocess import call, Popen, PIPE

import rospy
from dynamic_reconfigure.server import Server
from franka_teleoperation.cfg import workspace_scale_paramConfig
import tf.transformations
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64

from scipy.spatial.transform import Rotation as R
import numpy as np


# define the command_pose_index, command_pose_scale and slave_command_mode
slave_command_mode = None
command_pose_index, command_pose_scale = [], []


def dynamic_reconfigure_workspace_scale_param_callback(config, level):
    # declare the global variables
    global slave_command_mode, command_pose_scale
    
    # check the slave_command_mode(command_pose_scale order: ['x','y','z','rx','ry','rz'])
    if slave_command_mode == "absolute":
        command_pose_scale = [config.absolute_workspace_scale_x, config.absolute_workspace_scale_y, config.absolute_workspace_scale_z, 
                              config.absolute_workspace_scale_rx, config.absolute_workspace_scale_ry, config.absolute_workspace_scale_rz]
    elif slave_command_mode == "incremental":
        command_pose_scale = [config.incremental_workspace_scale_x, config.incremental_workspace_scale_y, config.incremental_workspace_scale_z, 
                              config.incremental_workspace_scale_rx, config.incremental_workspace_scale_ry, config.incremental_workspace_scale_rz]
    
    return config

def get_franka_pose():
    # get the state of franka
    msg = rospy.wait_for_message("/franka/franka_state_controller/franka_states", FrankaState)
    
    # parse the franka pose
    franka_quat = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4,4))))
    franka_quaternion = franka_quat / np.linalg.norm(franka_quat)
    
    # set the franka pose
    franka_pose = PoseStamped()
    franka_pose.header = msg.header
    franka_pose.pose.position.x = msg.O_T_EE[12]
    franka_pose.pose.position.y = msg.O_T_EE[13]
    franka_pose.pose.position.z = msg.O_T_EE[14]
    franka_pose.pose.orientation.x = franka_quaternion[0]
    franka_pose.pose.orientation.y = franka_quaternion[1]
    franka_pose.pose.orientation.z = franka_quaternion[2]
    franka_pose.pose.orientation.w = franka_quaternion[3]
    
    return franka_pose

def pose_mapping_callback(msg, args):
    # get the command_pose msgs and command_pose_scale dynamic parameters
    temp_command_pose = msg
    args[6] = command_pose_scale
    
    # update the slave robot(Franka) config pose for teleoperation for once if the touch button is pressed, otherwise keep the last saved one(in incremental mode)
    if temp_command_pose.pose.position.x == 0.0 and temp_command_pose.pose.position.y == 0.0 and temp_command_pose.pose.position.z == 0.0:
        if args[8] == True:
            args[7] = get_franka_pose()
            args[8] = False
    elif temp_command_pose.pose.position.x == -1.0 and temp_command_pose.pose.position.y == -1.0 and temp_command_pose.pose.position.z == -1.0:
        print("Resetting slave robot(Franka Panda/Gripper) and do not do any operation...\n")
        # TODO: reset the slave robot(Franka Panda/Gripper)
        Popen(["gnome-terminal", '--', 'sh', '-c', "rostopic pub --once /franka/franka_cartesian_impedance_controller/equilibrium_pose geometry_msgs/PoseStamped \
                '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'panda_hand_tcp'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}'"])
        time.sleep(10)  # wait 10 seconds for accomplishment of resetting the slave robot
        args[7] = get_franka_pose()
        print("Reset slave robot done and recovering to command mode...\n")
        return
    else:
        if args[8] == False:
            args[7] = get_franka_pose()
            args[8] = True
    
    # pose mapping according to the pose_mapping_order and absolute_workspace_scale/incremental_workspace_scale of master robot relative to slave robot
    if args[2] == "absolute":
        # set the header of command_pose msgs
        args[1].header.stamp = rospy.Time.now()
        args[1].header.frame_id = args[4]
        # mapping the coordinate and scale of command_pose msgs
        if args[3] == "position":
            # mapping position and keep slave robot config orientation
            args[1].pose.position.x = args[6][0] * eval("temp_command_pose.pose.position." + args[5][0])  # x
            args[1].pose.position.y = args[6][1] * eval("temp_command_pose.pose.position." + args[5][1])  # y
            args[1].pose.position.z = args[6][2] * eval("temp_command_pose.pose.position." + args[5][2])  # z
            args[1].pose.orientation = args[7].pose.orientation
        elif args[3] == "orientation":
            # mapping orientation and keep slave robot config position
            args[1].pose.position = args[7].pose.position
            master_quat = np.array([temp_command_pose.pose.orientation.x, temp_command_pose.pose.orientation.y, temp_command_pose.pose.orientation.z, temp_command_pose.pose.orientation.w])
            master_euler = R.from_quat(master_quat).as_euler(seq='xyz',degrees=True)
            slave_euler = np.array([args[6][3] * master_euler['xyz'.index(args[5][3])], args[6][4] * master_euler['xyz'.index(args[5][4])], args[6][5] * master_euler['xyz'.index(args[5][5])]])  # rx, ry, rz
            # slave_config_quat = np.array([args[7].pose.orientation.x, args[7].pose.orientation.y, args[7].pose.orientation.z, args[7].pose.orientation.w])
            # slave_quat = R.from_matrix(R.from_euler('xyz', slave_euler, degrees=True).as_matrix().dot(R.from_quat(slave_config_quat).as_matrix())).as_quat()
            slave_quat = R.from_euler('xyz', slave_euler, degrees=True).as_quat()
            args[1].pose.orientation.x = slave_quat[0]
            args[1].pose.orientation.y = slave_quat[1]
            args[1].pose.orientation.z = slave_quat[2]
            args[1].pose.orientation.w = slave_quat[3]
        elif args[3] == "pose":
            # mapping both position and orientation
            args[1].pose.position.x = args[6][0] * eval("temp_command_pose.pose.position." + args[5][0])  # x
            args[1].pose.position.y = args[6][1] * eval("temp_command_pose.pose.position." + args[5][1])  # y
            args[1].pose.position.z = args[6][2] * eval("temp_command_pose.pose.position." + args[5][2])  # z
            master_quat = np.array([temp_command_pose.pose.orientation.x, temp_command_pose.pose.orientation.y, temp_command_pose.pose.orientation.z, temp_command_pose.pose.orientation.w])
            master_euler = R.from_quat(master_quat).as_euler(seq='xyz',degrees=True)
            slave_euler = np.array([args[6][3] * master_euler['xyz'.index(args[5][3])], args[6][4] * master_euler['xyz'.index(args[5][4])], args[6][5] * master_euler['xyz'.index(args[5][5])]])  # rx, ry, rz
            # slave_config_quat = np.array([args[7].pose.orientation.x, args[7].pose.orientation.y, args[7].pose.orientation.z, args[7].pose.orientation.w])
            # slave_quat = R.from_matrix(R.from_euler('xyz', slave_euler, degrees=True).as_matrix().dot(R.from_quat(slave_config_quat).as_matrix())).as_quat()
            slave_quat = R.from_euler('xyz', slave_euler, degrees=True).as_quat()
            args[1].pose.orientation.x = slave_quat[0]
            args[1].pose.orientation.y = slave_quat[1]
            args[1].pose.orientation.z = slave_quat[2]
            args[1].pose.orientation.w = slave_quat[3]
        
    elif args[2] == "incremental":
        # set the header of command_pose msgs
        args[1].header.stamp = rospy.Time.now()
        args[1].header.frame_id = args[4]
        # mapping the coordinate and scale of command_pose msgs
        if args[3] == "position":
            # mapping position and keep slave robot config orientation
            args[1].pose.position.x = args[7].pose.position.x + args[6][0] * eval("temp_command_pose.pose.position." + args[5][0])  # x
            args[1].pose.position.y = args[7].pose.position.y + args[6][1] * eval("temp_command_pose.pose.position." + args[5][1])  # y
            args[1].pose.position.z = args[7].pose.position.z + args[6][2] * eval("temp_command_pose.pose.position." + args[5][2])  # z
            args[1].pose.orientation = args[7].pose.orientation
        elif args[3] == "orientation":
            # mapping orientation and keep slave robot config position
            args[1].pose.position = args[7].pose.position
            master_quat = np.array([temp_command_pose.pose.orientation.x, temp_command_pose.pose.orientation.y, temp_command_pose.pose.orientation.z, temp_command_pose.pose.orientation.w])
            master_euler = R.from_quat(master_quat).as_euler(seq='xyz',degrees=True)
            slave_euler = np.array([args[6][3] * master_euler['xyz'.index(args[5][3])], args[6][4] * master_euler['xyz'.index(args[5][4])], args[6][5] * master_euler['xyz'.index(args[5][5])]])  # rx, ry, rz
            slave_config_quat = np.array([args[7].pose.orientation.x, args[7].pose.orientation.y, args[7].pose.orientation.z, args[7].pose.orientation.w])
            slave_quat = R.from_matrix(R.from_euler('xyz', slave_euler, degrees=True).as_matrix().dot(R.from_quat(slave_config_quat).as_matrix())).as_quat()
            args[1].pose.orientation.x = slave_quat[0]
            args[1].pose.orientation.y = slave_quat[1]
            args[1].pose.orientation.z = slave_quat[2]
            args[1].pose.orientation.w = slave_quat[3]
        elif args[3] == "pose":
            # mapping both position and orientation
            args[1].pose.position.x = args[7].pose.position.x + args[6][0] * eval("temp_command_pose.pose.position." + args[5][0])  # x
            args[1].pose.position.y = args[7].pose.position.y + args[6][1] * eval("temp_command_pose.pose.position." + args[5][1])  # y
            args[1].pose.position.z = args[7].pose.position.z + args[6][2] * eval("temp_command_pose.pose.position." + args[5][2])  # z
            master_quat = np.array([temp_command_pose.pose.orientation.x, temp_command_pose.pose.orientation.y, temp_command_pose.pose.orientation.z, temp_command_pose.pose.orientation.w])
            master_euler = R.from_quat(master_quat).as_euler(seq='xyz',degrees=True)
            slave_euler = np.array([args[6][3] * master_euler['xyz'.index(args[5][3])], args[6][4] * master_euler['xyz'.index(args[5][4])], args[6][5] * master_euler['xyz'.index(args[5][5])]])  # rx, ry, rz
            slave_config_quat = np.array([args[7].pose.orientation.x, args[7].pose.orientation.y, args[7].pose.orientation.z, args[7].pose.orientation.w])
            slave_quat = R.from_matrix(R.from_euler('xyz', slave_euler, degrees=True).as_matrix().dot(R.from_quat(slave_config_quat).as_matrix())).as_quat()
            args[1].pose.orientation.x = slave_quat[0]
            args[1].pose.orientation.y = slave_quat[1]
            args[1].pose.orientation.z = slave_quat[2]
            args[1].pose.orientation.w = slave_quat[3]
    
    # publish the command_pose msgs
    args[0].publish(args[1])

def master_slave_pose_mapping_node():
    # declare the global variables
    global slave_command_mode, command_pose_index, command_pose_scale
    
    # init ros node
    rospy.init_node("master_slave_pose_mapping", anonymous=True)
    
    # get ros parameters from parameter server
    slave_command_mode = rospy.get_param("/franka/master_slave_pose_mapping/slave_command_mode")
    if slave_command_mode == "absolute":
        master_command_topic = "/touch/master_touch/command_pose/absolute"
        workspace_scale = rospy.get_param("/franka/master_slave_pose_mapping/absolute_workspace_scale")
    elif slave_command_mode == "incremental":
        master_command_topic = "/touch/master_touch/command_pose/incremental"
        workspace_scale = rospy.get_param("/franka/master_slave_pose_mapping/incremental_workspace_scale")
    slave_command_content = rospy.get_param("/franka/master_slave_pose_mapping/slave_command_content")
    slave_command_frame_id = rospy.get_param("/franka/master_slave_pose_mapping/slave_command_frame_id")
    pose_mapping_order = rospy.get_param("/franka/master_slave_pose_mapping/pose_mapping_order")
    
    # set the command_pose_index and command_pose_scale
    slave_robot_coordinate_index = ['x','y','z','rx','ry','rz']
    for idx in range(len(pose_mapping_order)):
        if 'r' not in pose_mapping_order[idx]:
            command_pose_index.append(pose_mapping_order[idx])
        elif 'r' in pose_mapping_order[idx]:
            command_pose_index.append(pose_mapping_order[idx].strip('r'))
    for item in slave_robot_coordinate_index:
        command_pose_scale.append(workspace_scale[pose_mapping_order.index(item)])
    
    # start the dynamic reconfigure parameter server
    dynamic_reconfigure_parameter_server = Server(workspace_scale_paramConfig, dynamic_reconfigure_workspace_scale_param_callback)
    
    # set the salve robot(Franka) config pose for teleoperation in incremental command mode
    slave_teleop_config_pose = PoseStamped()
    slave_teleop_config_pose = get_franka_pose()
    slave_teleop_config_flag = False
    
    # define the publisher and msgs for slave pose command
    if slave_command_mode == "absolute":
        pub_command_pose = rospy.Publisher("/franka/slave_franka/command_pose/absolute", PoseStamped, queue_size=1, tcp_nodelay=True, latch=False)
    elif slave_command_mode == "incremental":
        pub_command_pose = rospy.Publisher("/franka/slave_franka/command_pose/incremental", PoseStamped, queue_size=1, tcp_nodelay=True, latch=False)
    command_pose = PoseStamped()
    
    # define the subscriber for getting command pose from master touch(tip)
    sub_command_pose = rospy.Subscriber(master_command_topic, PoseStamped, 
                                        callback=pose_mapping_callback, 
                                        callback_args=[pub_command_pose, command_pose, slave_command_mode, slave_command_content, slave_command_frame_id, command_pose_index, command_pose_scale, slave_teleop_config_pose, slave_teleop_config_flag], 
                                        queue_size=1, 
                                        tcp_nodelay=True)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    # run ros node
    master_slave_pose_mapping_node()

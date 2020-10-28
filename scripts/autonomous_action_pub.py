#! /usr/bin/env python
import os
import sys
import argparse
import serial
import ctypes
import random
import multiprocessing
import rospy
import numpy as np
import time
import csv
from math import pi
from math import sin, cos
import datetime as dt

from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32MultiArray

ACTIONS =  {'r': [ 0, -1,  0,  0,  0,  0],
            'l': [ 0,  1,  0,  0,  0,  0],
            'f': [ 1,  0,  0,  0,  0,  0],
            'b': [-1,  0,  0,  0,  0,  0],
            'u': [ 0,  0,  1,  0,  0,  0],
            'd': [ 0,  0, -1,  0,  0,  0],
            'none': [ 0,  0,  0,  0,  0,  0]}
ACTION_KEYS = {'motion': ['r', 'l', 'f', 'b', 'u', 'd'], 'reset': 'n'}
ACTION_START_TIME = 0
NUM_EPOCH = 0
NUM_EPISODE = 0
LAST_ACTION = None

def weighted_sample_action():
    global ACTION_KEYS
    prob = np.array([2., 1., 1., 1., 2., 1.])
    index = np.random.choice(np.arange(6), p=prob/np.sum(prob))
    return ord(ACTION_KEYS['motion'][index])

def feedback_pose_abs(robot_name):
    feedback = rospy.wait_for_message("/" + robot_name + "/base_feedback", BaseCyclic_Feedback)
    base = feedback.base
    pose = [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z]
    pose = pose + [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]
    return pose

def force_cb(force, cb_args):
    global ACTIONS
    global ACTION_KEYS
    global LAST_ACTION
    args = cb_args[0]
    robot_name = cb_args[1]
    keyboard_pub = cb_args[2]
    keyboard_msg = cb_args[3]

    resistance = rospy.wait_for_message("/resistance", Int32)
    if resistance.data > 1:
        action = ord('n')
        keyboard_msg.data = action
        keyboard_pub.publish(keyboard_msg)
    else:
        action = weighted_sample_action()
        while not action_safety(args, action, robot_name):
            action = weighted_sample_action()
        keyboard_msg.data = action
        keyboard_pub.publish(keyboard_msg)

    LAST_ACTION = chr(action)

    print(type(force.data), type(force.data[0]))

    print('Resistance: ', resistance, 'Last action', chr(action))

def epoch_cb(data, cb_args):
    global ACTIONS
    global ACTION_KEYS
    global LAST_ACTION
    args = cb_args[0]
    robot_name = cb_args[1]
    keyboard_pub = cb_args[2]
    keyboard_msg = cb_args[3]
    # action = ord(random.sample(ACTION_KEYS['motion'], 1)[0])
    action = weighted_sample_action()
    while not action_safety(args, action, robot_name):
        action = weighted_sample_action()
    keyboard_msg.data = action
    keyboard_pub.publish(keyboard_msg)

def action_safety(args, action, robot_name):
    '''
    verify the safety of the action by computing the z positon
    '''
    global ACTIONS
    global ACTION_KEYS
    time_lim = args.time_lim
    zlb = args.zlb
    velocity = args.velocity
    dz = ACTIONS[chr(action)][2] * velocity * time_lim
    tool_z = feedback_pose_abs(robot_name)[2]
    if tool_z + dz < zlb:
        rospy.loginfo('Unsafe action, resampling...')
        return False
    else:
        return True
  
def main(args):
    global ACTIONS
    global ACTION_KEYS
    global ACTION_START_TIME 
    global NUM_EPOCH
    NUM_EPOCH = args.start_epoch
    global NUM_EPISODE
    global LAST_ACTION

    tool_start = [0.483,  0.066, 0.073]
    t_epoch = args.start_epoch
    t_episode = 0
    
    rospy.init_node('action_publisher')
    rate=rospy.Rate(20)

    keyboard_pub = rospy.Publisher('keyboard_pub', Int32, queue_size=1)
    keyboard_msg = Int32()

    robot_name = rospy.get_param('~robot_name',"my_gen3")
    rospy.Subscriber("load_cell_force", Float32MultiArray, force_cb, 
                        (args, robot_name, keyboard_pub, keyboard_msg))
    rospy.Subscriber("new_epoch_command", Int32, epoch_cb, 
                        (args, robot_name, keyboard_pub, keyboard_msg))


    while not rospy.is_shutdown():
        
        rate.sleep()

                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--output_dir', default='/home/jc/logs/realrobot', help='file to store openloop test results')
    parser.add_argument('--velocity', default=0.01, type=float, help='cartesian velocity of end effector')
    parser.add_argument('--time_lim', default=4, type=float, help='robot action magnitude')
    parser.add_argument('--zlb', default=0.05, type=float, help='lower bound of z for safe zone')
    parser.add_argument('--force_lim', default=200, type=float, help='tactile force limit')
    parser.add_argument('--start_epoch', default=0, type=int, help='number of epoch to start recording')
    args = parser.parse_args()
    
    main(args)
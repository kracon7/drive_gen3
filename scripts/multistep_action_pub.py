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
import numpy.linalg as la
import time
import csv
from math import pi
from math import sin, cos
import datetime as dt
from scipy.spatial.transform import Rotation as R

from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32MultiArray


ACTIONS =  {'r': np.array([ 0., -1.,  0.,  0.,  0.,  0.]),
            'l': np.array([ 0.,  1.,  0.,  0.,  0.,  0.]),
            'f': np.array([ 1.,  0.,  0.,  0.,  0.,  0.]),
            'b': np.array([-1.,  0.,  0.,  0.,  0.,  0.]),
            'u': np.array([ 0.,  0.,  1.,  0.,  0.,  0.]),
            'd': np.array([ 0.,  0., -1.,  0.,  0.,  0.]),
            'none': np.array([ 0.,  0.,  0.,  0.,  0.,  0.])}
ACTION_KEYS = {'motion': ['r', 'l', 'f', 'b', 'u', 'd'], 'reset': 'n'}
ACTION_START_TIME = 0
LAST_ACTION = None
EULER_TOOL = np.array([0., 0., 0.])
NUM_WEAK = 0

def select_optimal(robot_name):
    global ACTIONS
    global ACTION_KEYS
    pose = feedback_pose_abs(robot_name)[:3]
    vec = np.array([0.483, -0.15, 0.33]) - pose
    a = np.zeros([6, 3])
    for i in range(6):
        a[i] = ACTIONS[ACTION_KEYS['motion'][i]][:3]
    score = np.dot(a, vec)
    key = np.argmax(score)
    return ord(ACTION_KEYS['motion'][key])

def random_sample(robot_name):
    global ACTION_KEYS
    prob = np.array([1., .5, 2., 2., 1., .3])
    index = np.random.choice(np.arange(6), p=prob/np.sum(prob))
    return ord(ACTION_KEYS['motion'][index])

def weighted_sample_action(robot_name):
    global ACTIONS
    global ACTION_KEYS
    p = random.random()
    if p > 0.5:
        return select_optimal(robot_name)
    else:
        return random_sample(robot_name)
    

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
    global NUM_WEAK
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
        # check force.data and determine whether it's a weak force feedback
        force_mag = la.norm(np.array(force.data).reshape(-1, 3), axis=1)
        if np.amax(force_mag)**2 > (args.force_lim * 0.3):
            action = weighted_sample_action(robot_name)

            while not action_safety(args, action, robot_name):
                action = weighted_sample_action(robot_name)
            keyboard_msg.data = action
            keyboard_pub.publish(keyboard_msg)

            LAST_ACTION = action
        else:
            NUM_WEAK += 1
            if NUM_WEAK <= 3:
                print('Weak feedback, force magnitude is {}'.format(np.amax(force_mag)))
                action = LAST_ACTION  

                while not action_safety(args, action, robot_name):
                    action = weighted_sample_action(robot_name)
            else:
                NUM_WEAK = 0
                action = weighted_sample_action(robot_name)

                while not action_safety(args, action, robot_name):
                    action = weighted_sample_action(robot_name)
            keyboard_msg.data = action
            keyboard_pub.publish(keyboard_msg)

    print('Resistance: ', resistance, 'Last action', chr(action))

def epoch_cb(euler, cb_args):
    global ACTIONS
    global ACTION_KEYS
    global LAST_ACTION
    global EULER_TOOL

    EULER_TOOL = np.array(euler.data)
    rospy.loginfo('Recieved new euler tool, rotating action space....')
    rotate_action_space()

    args = cb_args[0]
    robot_name = cb_args[1]
    keyboard_pub = cb_args[2]
    keyboard_msg = cb_args[3]

    # action = ord(random.sample(ACTION_KEYS['motion'], 1)[0])
    action = weighted_sample_action(robot_name)
    while not action_safety(args, action, robot_name):
        action = weighted_sample_action(robot_name)
    keyboard_msg.data = action
    keyboard_pub.publish(keyboard_msg)
    LAST_ACTION = action

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

def rotate_action_space():
    global ACTIONS
    global EULER_TOOL
    ez, ey, ex = EULER_TOOL
    R_action = R.from_euler('zxy', [-ez, ey, ex], degrees=True).as_dcm()
    ACTIONS['r'][:3] = np.dot(R_action, np.array([0., -1., 0.]))
    ACTIONS['l'][:3] = np.dot(R_action, np.array([0.,  1., 0.]))
    ACTIONS['f'][:3] = np.dot(R_action, np.array([1.,  0., 0.]))
    ACTIONS['b'][:3] = np.dot(R_action, np.array([-1., 0., 0.]))
    ACTIONS['u'][:3] = np.dot(R_action, np.array([0.,  0., 1.]))
    ACTIONS['d'][:3] = np.dot(R_action, np.array([0.,  0., -1.]))

  
def main(args):
    global ACTIONS
    global ACTION_KEYS
    global ACTION_START_TIME 
    global LAST_ACTION

    t_epoch = args.start_epoch
    t_episode = 0
    
    rospy.init_node('action_publisher')
    rate=rospy.Rate(20)

    keyboard_pub = rospy.Publisher('keyboard_pub', Int32, queue_size=1)
    keyboard_msg = Int32()

    robot_name = rospy.get_param('~robot_name',"my_gen3")
    rospy.Subscriber("load_cell_force", Float32MultiArray, force_cb, 
                        (args, robot_name, keyboard_pub, keyboard_msg))
    rospy.Subscriber("new_epoch_command", Float32MultiArray, epoch_cb, 
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
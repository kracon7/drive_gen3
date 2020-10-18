#! /usr/bin/env python
import os
import sys
import argparse
import serial
import ctypes
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
from std_msgs.msg import Int32, Int16MultiArray

ACTIONS =  {'r': [ 0, -1,  0,  0,  0,  0],
            'l': [ 0,  1,  0,  0,  0,  0],
            'f': [ 1,  0,  0,  0,  0,  0],
            'b': [-1,  0,  0,  0,  0,  0],
            'u': [ 0,  0,  1,  0,  0,  0],
            'd': [ 0,  0, -1,  0,  0,  0],
            'none': [ 0,  0,  0,  0,  0,  0]}
ACTION_START_TIME = 0
ACTION_KEY = 'none'
NUM_EPOCH = 0
NUM_EPISODE = 0

def keyboard_cb(data, args):
    global ACTION_KEY
    global ACTION_START_TIME
    global NUM_EPOCH
    global NUM_EPISODE

    output_dir = args[0]
    audio_time = args[1]

    c = data.data   # c type is int
    if ACTIONS.has_key(chr(c)):
        rospy.loginfo('Recieved action command {}'.format(chr(c)))
        # call audio recording script
        cmd = 'python3 record_audio.py --epoch {} --episode {} -t {} --out_dir {}'.format(
                    NUM_EPOCH, NUM_EPISODE, audio_time, output_dir)
        os.system(cmd)
        # update action start time and number of episode
        ACTION_START_TIME = time.time()
        ACTION_KEY = chr(c)
        NUM_EPISODE += 1
    elif chr(c) == 'n':
        # new epoch, reset number of epoch and episode
        NUM_EPOCH += 1
        NUM_EPISODE = 0
    else:
        rospy.loginfo('ERROR: Unrecognized action command.')
        ACTION_KEY = 'none'

 
def main(args):
    global ACTION_KEY
    global ACTION_START_TIME
    global NUM_EPOCH
    global NUM_EPISODE

    t_epoch = 0
    t_episode = 0

    rospy.init_node('audio_recorder')
    rate=rospy.Rate(10)
    rospy.Subscriber("keyboard_pub", Int32, keyboard_cb, (args.output_dir, args.time_lim))
        
    while not rospy.is_shutdown():
        time_now = time.time()
        
        # if we are still within the operation time window
        if time_now > ACTION_START_TIME and time_now < (ACTION_START_TIME+args.time_lim):
            a=1

        if t_epoch == (NUM_EPOCH - 1) and NUM_EPISODE == 0:
            # write all csv rows from the last epoch
            rospy.loginfo('New epoch just begins, write csv and open a new one')
            
            t_epoch += 1
            t_episode = 0               

        if time_now > (ACTION_START_TIME+args.time_lim) and t_epoch == NUM_EPOCH \
                         and t_episode == (NUM_EPISODE -1):
            t_episode += 1

        print(t_epoch, NUM_EPOCH, t_episode, NUM_EPISODE)

        # twist_pub.publish(twist_msg)
        rate.sleep()

                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--output_dir', default='/home/jc/logs/realrobot', help='file to store openloop test results')
    parser.add_argument('--time_lim', default=5, type=float, help='robot action magnitude')
    args = parser.parse_args()
    
    main(args)

    


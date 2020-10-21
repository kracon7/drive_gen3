# ! /usr/bin/env python
import os
import sys
import argparse
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
ACTION_CSV = None
ACTION_CSV_WRITER = None

def feedback_pose_abs(robot_name):
    feedback = rospy.wait_for_message("/" + robot_name + "/base_feedback", BaseCyclic_Feedback)
    base = feedback.base
    pose = [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z]
    pose = pose + [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]
    return pose

def keyboard_cb(data, cb_args):
    global ACTION_KEY
    global ACTION_START_TIME
    global NUM_EPOCH
    global NUM_EPISODE
    global ACTION_CSV
    global ACTION_CSV_WRITER

    args = cb_args[0]
    robot_name = cb_args[1]
    output_dir = args.output_dir
    audio_time = args.time_lim
    zlb = args.zlb
    velocity = args.velocity
    tool_z = feedback_pose_abs(robot_name)[2]

    time_now = time.time()
    if time_now > ACTION_START_TIME and time_now < (ACTION_START_TIME+audio_time):
        rospy.loginfo('Actions must be given at least {}s after the last one'.format(audio_time))
    else:

        c = data.data   # c type is int
        if ACTIONS.has_key(chr(c)):
            rospy.loginfo('Recieved action command {}'.format(chr(c)))
            dz = ACTIONS[chr(c)][2] * velocity * audio_time
            if tool_z + dz > zlb:
                # update action start time and number of episode
                ACTION_START_TIME = time.time()
                ACTION_KEY = chr(c)
                NUM_EPISODE += 1

                ACTION_CSV_WRITER.writerow(chr(c))

                # call audio recording script
                cmd = 'python3 record_audio.py --epoch {} --episode {} -t {} --out_dir {}'.format(
                            NUM_EPOCH, NUM_EPISODE, audio_time, output_dir)
                os.system(cmd)
            else:
                rospy.loginfo('Action unsafe')
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
    NUM_EPOCH = args.start_epoch
    global NUM_EPISODE    
    global ACTION_CSV
    global ACTION_CSV_WRITER

    t_epoch = args.start_epoch
    t_episode = 0

    ACTION_CSV = open(os.path.join(args.output_dir, 'epoch_{}_action.csv'.format(NUM_EPOCH)), 'w')
    ACTION_CSV_WRITER = csv.writer(ACTION_CSV, delimiter=',')
    
    rospy.init_node('audio_recorder')
    rate=rospy.Rate(10)
    # check current tool pose and check validity of action
    robot_name = rospy.get_param('~robot_name',"my_gen3")
    tool_pose = feedback_pose_abs(robot_name)
    rospy.loginfo('Current tool pose: {}'.format(tool_pose))
    rospy.Subscriber("keyboard_pub", Int32, keyboard_cb, (args, robot_name))

        
    while not rospy.is_shutdown():
        time_now = time.time()

        if t_epoch == (NUM_EPOCH - 1) and NUM_EPISODE == 0:
            # write all csv rows from the last epoch
            rospy.loginfo('New epoch just begins, write csv and open a new one')
            # close previous csv file and open new a new one
            ACTION_CSV.close()
            ACTION_CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}_action.csv'.format(NUM_EPOCH)), 'w')
            ACTION_CSV_WRITER = csv.writer(ACTION_CSV, delimiter=',')
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
    parser.add_argument('--velocity', default=0.01, type=float, help='cartesian velocity of end effector')
    parser.add_argument('--time_lim', default=5, type=float, help='robot action magnitude')
    parser.add_argument('--zlb', default=0.05, type=float, help='lower bound of z for safe zone')
    parser.add_argument('--force_lim', default=250, type=float, help='tactile force limit')
    parser.add_argument('--start_epoch', default=0, type=int, help='number of epoch to start recording')
    args = parser.parse_args()
    
    main(args)

    


#! /usr/bin/env python
import os
import sys
import argparse
import serial
import ctypes
import multiprocessing
import rospy
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Int16MultiArray
from math import pi
from math import sin, cos
import time

FORCE_LOG = [[0., 0., 0.]]
# set up serial port
port = '/dev/ttyACM0'
arduino_port = serial.Serial(port,9600,timeout=5)
shared_array_base = multiprocessing.Array(ctypes.c_double, 3*1)
shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
shared_array = shared_array.reshape(1, 3)


def write_twist_msg(twist_msg, ref_frame=0, twist=[0.,0.,0.,0.,0.,0.], duration=0):
    twist_msg.reference_frame = ref_frame
    twist_msg.twist.linear_x = twist[0]
    twist_msg.twist.linear_y = twist[1]
    twist_msg.twist.linear_z = twist[2]
    twist_msg.twist.angular_x = twist[3]
    twist_msg.twist.angular_y = twist[4]
    twist_msg.twist.angular_z = twist[5]
    twist_msg.duration = duration

def read_force(arduino_port):
    # read message from arduino port to extract forces and cast into float
    msg = ''
    while 'HX711' in msg or 'resistance' in msg or len(msg)<10 or len(msg.split())!=4:
        msg = arduino_port.readline()
    forces = msg.split()[1:]
    rospy.loginfo('Arduino message: '+msg.rstrip())
    fx, fy, fz = forces
    return [float(fx), float(fy), float(fz)]

def read_resistance(arduino_port):
    msg = ''
    while 'HX711' in msg or 'force' in msg or len(msg)<5 or len(msg.split())!=2:
        msg = arduino_port.readline()
    resistance = msg.split()[1]
    return float(resistance)

def arduino_process(def_param=shared_array):
    while True:
        fx, fy, fz = read_force(arduino_port)
        shared_array[0,0] = fx
        shared_array[0,1] = fy
        shared_array[0,2] = fz

def feedback_pose_abs(robot_name):
    feedback = rospy.wait_for_message("/" + robot_name + "/base_feedback", BaseCyclic_Feedback)
    base = feedback.base
    pose = [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z]
    pose = pose + [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]
    return pose
        
def main(args):
    '''
    First,
    Initialize the Gen3 cartesian space controler

    '''
    rospy.init_node('demo_velocity_controller')
    rate=rospy.Rate(5)
    twist_pub = rospy.Publisher('my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
    twist_msg = TwistCommand()
    
    p = multiprocessing.Process(target=arduino_process, args=())
    p.start()
    time.sleep(4)

    robot_name = rospy.get_param('~robot_name',"my_gen3")
    tool_pose = feedback_pose_abs(robot_name)
    rospy.loginfo('Current tool pose: {}'.format(tool_pose))

    time_start = time.time()

    while not rospy.is_shutdown():
        time_now = time.time()
        # read force sensor message
        fx, fy, fz = shared_array[-1]
        force_mag = fx**2 + fy**2 + fz**2
        rospy.loginfo('Force magnitude: {}'.format(force_mag))

        # write_twist_msg(twist_msg, 0, [0.01, 0.01, 0.01, 0., 0., 0.], 0)

        # move end effector when t < 2
        if time_now - time_start < 15:
            rospy.loginfo('Moving End Effector.')
            write_twist_msg(twist_msg, 0, [0., 0.0, 0.01, 0., 0., 0.], 0)
            tool_pose = feedback_pose_abs(robot_name)
            rospy.loginfo('Current tool pose: {}'.format(tool_pose))

        # stop the ene effector
        else:
            rospy.loginfo('Stopped End Effector.')
            write_twist_msg(twist_msg, 0, [0, 0, 0, 0., 0., 0.], 0)
        
        if force_mag > 7:
            write_twist_msg(twist_msg, 0, [0, 0, 0, 0., 0., 0.], 0)

        twist_pub.publish(twist_msg)
        rate.sleep()

    p.terminate()
                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--act_mag', default=0.015, type=float, help='robot action magnitude')
    parser.add_argument('--zlb', default=0.02, type=float, help='lower bound of z for safe zone')
    parser.add_argument('--output_path', default='/home/jc/logs/realrobot/traj_log_1.txt', help='file to store openloop test results')
    args = parser.parse_args()
    
    main(args)

    


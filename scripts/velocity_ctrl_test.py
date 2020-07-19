#! /usr/bin/env python
import os
import sys
import argparse

import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Int16MultiArray
from math import pi
from math import sin, cos
import time

             
        
def main(args):
    '''
    First,
    Initialize the Gen3 cartesian space controler

    '''
    rospy.init_node('demo_velocity_controller')
    rate=rospy.Rate(50)
    twist_pub = rospy.Publisher('my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
    twist_msg = TwistCommand()
    
    time_start = time.time()

    while not rospy.is_shutdown():
        time_now = time.time()
        # move end effector when t < 2
        if time_now - time_start < 2:
            rospy.loginfo('Moving End Effector.')
            twist_msg.reference_frame = 0
            twist_msg.twist.linear_x = 0.02
            twist_msg.twist.linear_y = 0.02
            twist_msg.twist.linear_z = 0.02
            twist_msg.twist.angular_x = 0
            twist_msg.twist.angular_y = 0
            twist_msg.twist.angular_z = 0
            twist_msg.duration = 0 
            twist_pub.publish(twist_msg)
        # stop the ene effector
        else:
            rospy.loginfo('Stopped End Effector.')
            twist_msg.reference_frame = 0
            twist_msg.twist.linear_x = 0
            twist_msg.twist.linear_y = 0
            twist_msg.twist.linear_z = 0
            twist_msg.twist.angular_x = 0
            twist_msg.twist.angular_y = 0
            twist_msg.twist.angular_z = 0
            twist_msg.duration = 0 
            twist_pub.publish(twist_msg)
        rate.sleep()

    print('here')
                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--act_mag', default=0.015, type=float, help='robot action magnitude')
    parser.add_argument('--zlb', default=0.02, type=float, help='lower bound of z for safe zone')
    parser.add_argument('--output_path', default='/home/jc/logs/realrobot/traj_log_1.txt', help='file to store openloop test results')
    args = parser.parse_args()
    
    main(args)

    


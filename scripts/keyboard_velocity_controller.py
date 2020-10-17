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
CSV_FILE = None
CSV_WRITER = None

# set up serial port
port = '/dev/ttyACM0'
arduino_port = serial.Serial(port,9600,timeout=5)
shared_array_base = multiprocessing.Array(ctypes.c_double, 3*1)
shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
shared_array = shared_array.reshape(1, 3)

def read_force(arduino_port):
    # read message from arduino port to extract forces and cast into float
    msg = ''
    while 'HX711' in msg or 'resistance' in msg or len(msg)<10 or len(msg.split())!=4:
        msg = arduino_port.readline()
    forces = msg.split()[1:]
    # rospy.loginfo('Arduino message: '+msg.rstrip())
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

def keyboard_cb(data, output_dir):
    global ACTION_KEY
    global ACTION_START_TIME
    global NUM_EPOCH
    global NUM_EPISODE
    global CSV_FILE
    global CSV_WRITER

    c = data.data   # c type is int
    if ACTIONS.has_key(chr(c)):
        rospy.loginfo('Recieved action command {}'.format(chr(c)))
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

def write_twist_msg(twist_msg, ref_frame=0, twist=[0.,0.,0.,0.,0.,0.], duration=0):
    twist_msg.reference_frame = ref_frame
    twist_msg.twist.linear_x = twist[0]
    twist_msg.twist.linear_y = twist[1]
    twist_msg.twist.linear_z = twist[2]
    twist_msg.twist.angular_x = twist[3]
    twist_msg.twist.angular_y = twist[4]
    twist_msg.twist.angular_z = twist[5]
    twist_msg.duration = duration
  

class ArmController:
    
    def __init__(self, args):
    
        rospy.init_node('demo_arm_controller')
        
        self.HOME_ACTION_IDENTIFIER = 2
        self.HOME_CONFIG = [0,15,180,230,0,55,90]
        self.config = [0,15,180,230,0,55,90]
        
        self.args = args
        
        self.robot_name = rospy.get_param('~robot_name',"my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)
        
        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))
        
        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
        
        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
        
        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
        rospy.wait_for_service(play_cartesian_trajectory_full_name)
        self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)
        
        play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
        rospy.wait_for_service(play_joint_trajectory_full_name)
        self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)
        
        rospy.loginfo("Initialization Done..")


    def clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)


    def home_the_robot(self,sleep_time=10, init_gripper=False):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
                rospy.sleep(sleep_time)
                if init_gripper:
                    self.send_gripper_status(0)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")


    def set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        # req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        # req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)

    def send_cartesian_pose_abs(self, pose, pose_theta, speed_trans=0.1, speed_orient=15, sleep_time=10):
    
        # Send the abusolute pose related to the base_link
        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = pose[0]
        req.input.target_pose.y = pose[1]
        req.input.target_pose.z = pose[2]
        req.input.target_pose.theta_x = pose_theta[0]
        req.input.target_pose.theta_y = pose_theta[1]
        req.input.target_pose.theta_z = pose_theta[2]

        pose_speed = CartesianSpeed()
        pose_speed.translation = speed_trans
        pose_speed.orientation = speed_orient

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
            rospy.sleep(sleep_time)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")


    def send_cartesian_pose(self, pose, pose_theta, speed_trans=0.1, speed_orient=15, sleep_time=10):
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = feedback.base.commanded_tool_pose_x + pose[0]
        req.input.target_pose.y = feedback.base.commanded_tool_pose_y + pose[1]
        req.input.target_pose.z = feedback.base.commanded_tool_pose_z + pose[2]
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x + pose_theta[0]
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y + pose_theta[1]
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z + pose_theta[2]

        pose_speed = CartesianSpeed()
        pose_speed.translation = speed_trans
        pose_speed.orientation = speed_orient

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
            rospy.sleep(sleep_time)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            
            
    def send_joint_angles_abs(self,config,sleep_time=10):
        # Create the list of angles
        req = PlayJointTrajectoryRequest()

        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = config[i]
            req.input.joint_angles.joint_angles.append(temp_angle)
        
        # Send the angles
        rospy.loginfo("Sending the robot to config (%s,%s,%s,%s,%s,%s,%s)",
                    config[0],config[1],config[2],config[3],config[4],config[5],config[6])
        try:
            self.play_joint_trajectory(req)
            rospy.sleep(sleep_time)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            

    def send_joint_angles_rot(self,config_r,sleep_time=10):
        # Rotate joint angels with specific config
        req = PlayJointTrajectoryRequest()
        
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = self.feedback_config()[i] + config_r[i]  
            req.input.joint_angles.joint_angles.append(temp_angle)
        
        # Send the angles
        rospy.loginfo("Rotating the robot by (%s,%s,%s,%s,%s,%s,%s)",
                    config_r[0],config_r[1],config_r[2],config_r[3],config_r[4],config_r[5],config_r[6])
        try:
            self.play_joint_trajectory(req)
            rospy.sleep(sleep_time)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")

    
    def send_gripper_status(self, value, sleep_time=2):
        # Initialize the request
        # This works for the Robotiq Gripper 2F_85
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
            rospy.sleep(sleep_time)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
        

    def feedback_config(self):
        feedback_config = self.config
        feedback_joint = rospy.wait_for_message("/" + self.robot_name + "/joint_states", JointState)
        
        # Convert to degrees
        for i in range(self.degrees_of_freedom):
            feedback_config[i] = feedback_joint.position[i]*180/pi
            while feedback_config[i] < 0 or feedback_config[i] > 360:
                if feedback_config[i] < 0:
                    feedback_config[i] = feedback_config[i] + 360
                elif feedback_config[i] > 360:
                    feedback_config[i] = feedback_config[i] - 360
                    
        self.config = feedback_config       
        return feedback_config
        
    def feedback_pose_abs(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        base = feedback.base
        pose = [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z]
        pose = pose + [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]
        return pose


    def keyboard_cb(self, data, output_path):
        # # Robot cartesian space controller
        # global ex
        c = data.data   # c type is int
        if chr(c) in self.action_keys.keys():
            current_pose = self.feedback_pose_abs()

            key = self.action_keys[chr(c)]
            dp = self.action_space.actions[key]
            theta = [0, 0, 0]

            if current_pose[2] + dp[2] < args.zlb:
                print('Target pose unsafe!')
            else:
                # log time and ee_pose before action starts
                now = dt.datetime.now()
                with open(output_path, 'a') as f:
                    f.write('{} {}:{}:{},{} action {} starts\n'.format(now.day, now.hour, now.minute, now.second, now.microsecond, key))
                    f.write('                   End effector now at {}\n'.format(current_pose))
                f.close()

                # execute action
                self.send_cartesian_pose(dp, theta, speed_trans=0.03, speed_orient=5, sleep_time=1.5)
                rospy.loginfo("Finished sending the robot to the cartesian pose")  

                # log time and ee_pose again
                current_pose = self.feedback_pose_abs()
                now = dt.datetime.now()                
                with open(output_path, 'a') as f:
                    f.write('{} {}:{}:{},{} finished.\n'.format(now.day, now.hour, now.minute, now.second,now.microsecond, key))
                    f.write('                   End effector now at {}\n'.format(current_pose))
                f.close()

        # Slippage time log
        elif chr(c) is 'v':
            rospy.loginfo("Slippage message recieved!")  
            now = dt.datetime.now()
            with open(output_path, 'a') as f:
                f.write('{} {}:{}:{},{} Slipped!\n\n'.format(now.day, now.hour, now.minute, now.second, now.microsecond))
            f.close()

        elif chr(c) is 't':
            rospy.loginfo("Termination message recieved!")
            now = dt.datetime.now()
            with open(output_path, 'a') as f:
                f.write('{} {}:{}:{},{} Terminated!\n\n'.format(now.day, now.hour, now.minute, now.second, now.microsecond))
            f.close()
        else:
            print('No such action key available!')

  
def main(args):
    '''
    First,
    Initialize the Gen3 cartesian space controler

    '''
    # ex = ArmController(args)
    # ex.clear_faults()
    # ex.set_cartesian_reference_frame()   
    # task_home = [0.351, 0, 0.025, 180, 0, 90]
    # ex.send_cartesian_pose_abs(task_home[:3], task_home[3:],sleep_time=10)
    # rospy.loginfo("Finished sending the robot to the cartesian pose")   

    global ACTION_KEY
    global ACTION_START_TIME 
    global NUM_EPOCH
    global NUM_EPISODE
    global CSV_FILE
    global CSV_WRITER

    csv_all_rows = []
    csv_row = []
    t_epoch = 0
    t_episode = 0
    CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}.csv'.format(NUM_EPOCH)), 'w')
    CSV_WRITER = csv.writer(CSV_FILE, delimiter=',')

    rospy.init_node('velocity_controller')
    rate=rospy.Rate(10)
    twist_pub = rospy.Publisher('my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
    rospy.Subscriber("keyboard_pub", Int32, keyboard_cb, args.output_dir)
    twist_msg = TwistCommand()
    
    # turn on the arduino process for force and resistance logging
    p = multiprocessing.Process(target=arduino_process, args=())
    p.start()
    time.sleep(4)

    # check current tool pose and check validity of action
    robot_name = rospy.get_param('~robot_name',"my_gen3")
    tool_pose = feedback_pose_abs(robot_name)
    rospy.loginfo('Current tool pose: {}'.format(tool_pose))
        
    while not rospy.is_shutdown():
        time_now = time.time()
        
        # if we are still within the operation time window
        if time_now > ACTION_START_TIME and time_now < (ACTION_START_TIME+args.time_lim):
            # check force magnitude
            # read force sensor message
            fx, fy, fz = shared_array[-1]
            force_mag = fx**2 + fy**2 + fz**2
            # if force exceed limit, send zero velocity command
            if force_mag > args.force_lim:
                twist = args.velocity * np.array(ACTIONS['none'])
                write_twist_msg(twist_msg, 0, twist, 0)
                twist_pub.publish(twist_msg)
                continue
            else:
                twist = args.velocity * np.array(ACTIONS[ACTION_KEY])
                write_twist_msg(twist_msg, 0, twist, 0)
                twist_pub.publish(twist_msg)

            csv_row.append('{} {} {}'.format(fx, fy, fz))

        # publish zero twist if the time is outside the operation time window
        else:
            ACTION_KEY = 'none'
            twist = args.velocity * np.array(ACTIONS[ACTION_KEY])
            write_twist_msg(twist_msg, 0, twist, 0)
            twist_pub.publish(twist_msg)

        if t_epoch == (NUM_EPOCH - 1) and NUM_EPISODE == 0:
            # write all csv rows from the last epoch
            rospy.loginfo('New epoch just begins, write csv and open a new one')
            for row in csv_all_rows:
                CSV_WRITER.writerow(row)
            # close previous csv file and open new a new one
            CSV_FILE.close()
            CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}.csv'.format(NUM_EPOCH)), 'w')
            CSV_WRITER = csv.writer(CSV_FILE, delimiter=',')
            
            t_epoch += 1
            t_episode = 0               

        if time_now > (ACTION_START_TIME+args.time_lim) and t_epoch == NUM_EPOCH \
                         and t_episode == (NUM_EPISODE -1):
            print('writing new csv row')
            csv_all_rows.append(csv_row)
            # clear csv row for next episode
            csv_row = []
            t_episode += 1

        print(t_epoch, NUM_EPOCH, t_episode, NUM_EPISODE)

        # twist_pub.publish(twist_msg)
        rate.sleep()

                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--act_mag', default=0.015, type=float, help='robot action magnitude')
    parser.add_argument('--output_dir', default='/home/jc/logs/realrobot', help='file to store openloop test results')
    parser.add_argument('--velocity', default=0.01, type=float, help='cartesian velocity of end effector')
    parser.add_argument('--time_lim', default=5, type=float, help='robot action magnitude')
    parser.add_argument('--zlb', default=0.04, type=float, help='lower bound of z for safe zone')
    parser.add_argument('--force_lim', default=4, type=float, help='tactile force limit')
    args = parser.parse_args()
    
    main(args)

    


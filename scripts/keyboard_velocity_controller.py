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
ACTION_START_TIME = 0
ACTION_KEY = 'none'
NUM_EPOCH = 0
NUM_EPISODE = 0
FORCE_CSV_FILE = None
FORCE_CSV_WRITER = None
TRAJ_CSV_FILE = None
TRAJ_CSV_WRITER = None
FAILED_ACTIONS = None

# set up serial port
port = '/dev/ttyACM0'
arduino_port = serial.Serial(port,9600,timeout=5)
shared_array_base = multiprocessing.Array(ctypes.c_double, 4*1)
shared_array = np.ctypeslib.as_array(shared_array_base.get_obj())
shared_array = shared_array.reshape(1, 4)

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
        resistance = read_resistance(arduino_port)
        shared_array[0,0] = fx
        shared_array[0,1] = fy
        shared_array[0,2] = fz
        shared_array[0,3] = resistance

def feedback_pose_abs(robot_name):
    feedback = rospy.wait_for_message("/" + robot_name + "/base_feedback", BaseCyclic_Feedback)
    base = feedback.base
    pose = [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z]
    pose = pose + [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]
    return pose

def write_twist_msg(twist_msg, ref_frame=0, twist=[0.,0.,0.,0.,0.,0.], duration=0):
    twist_msg.reference_frame = ref_frame
    twist_msg.twist.linear_x = twist[0]
    twist_msg.twist.linear_y = twist[1]
    twist_msg.twist.linear_z = twist[2]
    twist_msg.twist.angular_x = twist[3]
    twist_msg.twist.angular_y = twist[4]
    twist_msg.twist.angular_z = twist[5]
    twist_msg.duration = duration

def keyboard_cb(data, cb_args):
    global ACTIONS
    global ACTION_KEY
    global ACTION_START_TIME
    global NUM_EPOCH
    global NUM_EPISODE
    global FORCE_CSV_FILE
    global FORCE_CSV_WRITER

    args = cb_args[0]
    time_lim = args.time_lim
    zlb = args.zlb
    velocity = args.velocity
    ex = cb_args[1]
    tool_z = ex.feedback_pose_abs()[2]

    time_now = time.time()
    if time_now > ACTION_START_TIME and time_now < (ACTION_START_TIME+time_lim):
        rospy.loginfo('Actions must be given at least {}s after the last one'.format(time_lim))
    else:
        c = data.data   # c type is int
        if ACTIONS.has_key(chr(c)):
            rospy.loginfo('Recieved action command {}'.format(chr(c)))
            dz = ACTIONS[chr(c)][2] * velocity * time_lim
            if tool_z + dz > zlb:
                rospy.loginfo('Valid action')
                ACTION_START_TIME = time.time()
                ACTION_KEY = chr(c)
                NUM_EPISODE += 1
            else:
                rospy.loginfo('Action unsafe')
        elif chr(c) == 'n':
            # new epoch, reset number of epoch and episode
            NUM_EPOCH += 1
            NUM_EPISODE = 0
        else:
            rospy.loginfo('ERROR: Unrecognized action command.')
            ACTION_KEY = 'none'
 
def reset_velcro(ex, new_epoch_pub, new_epoch_msg,
                 task_home=[0.483, -0.24, 0.27, 180, 0, 90]):
    user_response = raw_input("Send tool to task home position? y/n")
    if user_response == 'y' or user_response == 'Y':
        # random initialize the tool rotation angles in base frame
        # euler_tool: ez, ey, ex
        case = 2
        if case == 1:
            euler_tool = [0., 0., 0.]
        elif case == 2:
            euler_tool = [120 * (np.random.rand()), 
                          60 * (np.random.rand() - 0.5), 
                          60 * (np.random.rand() - 0.5)]
        elif case == 3:
            euler_tool = [90*np.random.rand(), 30*np.random.rand(), 30*np.random.rand()]
        rospy.loginfo('New tool rotation: {}'.format(euler_tool))
        euler_base = euler_tool_to_base(euler_tool)
        # send tool to task start in canonical form and then rotate
        ex.send_cartesian_pose_abs(task_home[:3], task_home[3:],sleep_time=8)
        pose_sequence = [[0.483, -0.158, 0.251],
                         [0.483, -0.076, 0.251],
                         [0.483, -0.026, 0.180],
                         [0.483,  0.046, 0.161],
                         [0.483,  0.063, 0.131],
                         [0.483,  0.071, 0.105],
                         [0.483,  0.086, 0.091],
                         [0.483,  0.091, 0.06],
                         [0.483,  0.091, 0.04],
                         [0.483,  0.066, 0.073]]
        for pose in pose_sequence:
            ex.send_cartesian_pose_abs(pose, task_home[3:], sleep_time=1.5)
        ex.send_cartesian_pose_abs(pose_sequence[-1], euler_base, sleep_time=5)

        # rotate action space
        rotate_action_space(euler_tool)

        # send user command to start new epoch
        user_response = raw_input("Press Enter to start new epoch")
        new_epoch_msg.data = euler_tool
        new_epoch_pub.publish(new_epoch_msg)

def euler_tool_to_base(euler_tool):
    R_be = R.from_euler('xyz', [180, 0, 90], degrees=True).as_dcm()
    R_eep = R.from_euler('zyx', euler_tool, degrees=True).as_dcm()
    R_bep = np.dot(R_be, R_eep)
    euler_base = R.from_dcm(R_bep).as_euler('xyz', degrees=True)
    return euler_base

def rotate_action_space(euler_tool):
    global ACTIONS
    ez, ey, ex = euler_tool
    R_action = R.from_euler('zxy', [-ez, ey, ex], degrees=True).as_dcm()
    ACTIONS['r'][:3] = np.dot(R_action, np.array([0., -1., 0.]))
    ACTIONS['l'][:3] = np.dot(R_action, np.array([0.,  1., 0.]))
    ACTIONS['f'][:3] = np.dot(R_action, np.array([1.,  0., 0.]))
    ACTIONS['b'][:3] = np.dot(R_action, np.array([-1., 0., 0.]))
    ACTIONS['u'][:3] = np.dot(R_action, np.array([0.,  0., 1.]))
    ACTIONS['d'][:3] = np.dot(R_action, np.array([0.,  0., -1.]))
    print(ACTIONS)


def publish_action(twist_pub, twist_msg, velocity, action_key):
    global ACTIONS
    twist = velocity * np.array(ACTIONS[action_key])
    write_twist_msg(twist_msg, 0, twist, 0)
    twist_pub.publish(twist_msg)

def to_force_msg(csv_list):
    '''
    convert the list of csv blocks to ROS force message
    Input:
        csv_list -- list of strings ['1 2 3', '0 1 2']
    '''
    msg = []
    for item in csv_list:
        fx, fy, fz = item.split()
        msg += [float(fx), float(fy), float(fz)]
    return msg


class ArmController:
    
    def __init__(self, args):
        
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

    def send_cartesian_pose_abs(self, pose, pose_theta, speed_trans=0.1, speed_orient=30, sleep_time=10):
    
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

  
def main(args):
    '''
    First,
    Initialize the Gen3 cartesian space controler

    '''
    ex = ArmController(args)
    ex.clear_faults()
    ex.set_cartesian_reference_frame()

    global ACTION_KEY
    global ACTION_START_TIME 
    global NUM_EPOCH
    NUM_EPOCH = args.start_epoch
    global NUM_EPISODE
    global FORCE_CSV_FILE
    global FORCE_CSV_WRITER
    global TRAJ_CSV_FILE
    global TRAJ_CSV_WRITER
    global FAILED_ACTIONS
    FAILED_ACTIONS = {'key': ['none', 'none', 'none'], 
            'force_mag': [args.force_lim, args.force_lim, args.force_lim]}

    tool_start = [0.483,  0.066, 0.073]
    force_csv_all_rows = []
    force_csv_row = []
    traj_csv_all_rows = [tool_start]
    t_epoch = args.start_epoch
    t_episode = 0
    FORCE_CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}_force.csv'.format(NUM_EPOCH)), 'w')
    FORCE_CSV_WRITER = csv.writer(FORCE_CSV_FILE, delimiter=',')
    TRAJ_CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}_traj.csv'.format(NUM_EPOCH)), 'w')
    TRAJ_CSV_WRITER = csv.writer(TRAJ_CSV_FILE, delimiter=',')

    rospy.init_node('velocity_controller')
    rate=rospy.Rate(10)
    twist_pub = rospy.Publisher('my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
    rospy.Subscriber("keyboard_pub", Int32, keyboard_cb, (args, ex))
    twist_msg = TwistCommand()

    force_pub = rospy.Publisher('load_cell_force', Float32MultiArray, queue_size=1)
    force_msg = Float32MultiArray()
    
    resistance_pub = rospy.Publisher('resistance', Int32, queue_size=1)
    resistance_msg = Int32()

    new_epoch_pub = rospy.Publisher('new_epoch_command', Float32MultiArray, queue_size=1)
    new_epoch_msg = Float32MultiArray()
    
    # turn on the arduino process for force and resistance logging
    p = multiprocessing.Process(target=arduino_process, args=())
    p.deamon = True
    p.start()
    time.sleep(4)

    reset_velcro(ex, new_epoch_pub, new_epoch_msg)

    # check current tool pose and check validity of action
    robot_name = rospy.get_param('~robot_name',"my_gen3")
    tool_pose = feedback_pose_abs(robot_name)
    rospy.loginfo('Current tool pose: {}'.format(tool_pose))
        
    while not rospy.is_shutdown():
        time_now = time.time()
        # check force magnitude
        # read force sensor message
        fx, fy, fz, resistance = shared_array[-1]
        force_mag = fx**2 + fy**2 + fz**2

        resistance_msg.data = int(resistance)
        resistance_pub.publish(resistance_msg)

        # if we are still within the operation time window
        if time_now > ACTION_START_TIME and time_now < (ACTION_START_TIME+args.time_lim):
            # if force exceed limit, send zero velocity command
            if force_mag > args.force_lim and ACTION_KEY in FAILED_ACTIONS['key']:
                # find the last failed attemp and compare the force magnitude
                # continue with the action if the force_mag is smaller than last time
                key_index = [i for i, x in enumerate(FAILED_ACTIONS['key']) if x == ACTION_KEY] 
                key_forces = [FAILED_ACTIONS['force_mag'][i] for i in key_index]
                last_force_mag = min(key_forces)
                print(force_mag, last_force_mag)
                if force_mag < min(0.75 * last_force_mag, args.force_lim):
                    # print(force_mag, last_force_mag)
                    publish_action(twist_pub, twist_msg, args.velocity, ACTION_KEY)
                else:
                    publish_action(twist_pub, twist_msg, args.velocity, 'none')
            elif force_mag > args.force_lim and ACTION_KEY not in FAILED_ACTIONS['key']:
                if time_now < (ACTION_START_TIME + 0.2):
                    publish_action(twist_pub, twist_msg, args.velocity, ACTION_KEY)
                else:
                    publish_action(twist_pub, twist_msg, args.velocity, 'none')
            else:
                publish_action(twist_pub, twist_msg, args.velocity, ACTION_KEY)

            force_csv_row.append('{} {} {}'.format(fx, fy, fz))

        # publish zero twist if the time is outside the operation time window
        else:
            if ACTION_KEY is not 'none' and force_mag > args.force_lim:
                # update the new failed action
                FAILED_ACTIONS['key'].pop(0)
                FAILED_ACTIONS['key'].append(ACTION_KEY)
                FAILED_ACTIONS['force_mag'].pop(0)
                FAILED_ACTIONS['force_mag'].append(force_mag)               

            ACTION_KEY = 'none'
            publish_action(twist_pub, twist_msg, args.velocity, 'none')

        # new epoch
        if t_epoch == (NUM_EPOCH - 1) and NUM_EPISODE == 0:
            # write all csv rows from the last epoch
            rospy.loginfo('New epoch just begins, write csv and open a new one')
            for row in force_csv_all_rows:
                FORCE_CSV_WRITER.writerow(row)
            # close previous csv file and open new a new one
            FORCE_CSV_FILE.close()
            FORCE_CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}_force.csv'.format(NUM_EPOCH)), 'w')
            FORCE_CSV_WRITER = csv.writer(FORCE_CSV_FILE, delimiter=',')
            force_csv_all_rows = []

            for row in traj_csv_all_rows:
                TRAJ_CSV_WRITER.writerow(row)
            TRAJ_CSV_FILE.close()
            TRAJ_CSV_FILE = open(os.path.join(args.output_dir, 'epoch_{}_traj.csv'.format(NUM_EPOCH)), 'w')
            TRAJ_CSV_WRITER = csv.writer(TRAJ_CSV_FILE, delimiter=',')
            traj_csv_all_rows = [tool_start]

            # reset the velcro by controlling the tool following a fixed trajectory
            reset_velcro(ex, new_epoch_pub, new_epoch_msg)
            
            t_epoch += 1
            t_episode = 0  

            # reset unsafe actions
            FAILED_ACTIONS = {'key': ['none', 'none', 'none'], 
                    'force_mag': [args.force_lim, args.force_lim, args.force_lim]}             

        # new episode
        if time_now > (ACTION_START_TIME+args.time_lim) and t_epoch == NUM_EPOCH \
                         and t_episode == (NUM_EPISODE -1):
            print('writing new csv row')
            # publish force data
            force_msg.data = to_force_msg(force_csv_row)
            force_pub.publish(force_msg)

            force_csv_all_rows.append(force_csv_row)
            # clear csv row for next episode
            force_csv_row = []

            traj_csv_all_rows.append(ex.feedback_pose_abs()[:3])

            t_episode += 1

        print(t_epoch, NUM_EPOCH, t_episode, NUM_EPISODE)
        print(FAILED_ACTIONS['key'], FAILED_ACTIONS['force_mag'])

        # twist_pub.publish(twist_msg)
        rate.sleep()

    p.terminate()
        
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
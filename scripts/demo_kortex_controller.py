#! /usr/bin/env python

import sys
import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
from math import pi

class ArmControllerDemo:
	
	def __init__(self):
	
		rospy.init_node('demo_arm_controller')
		
		self.HOME_ACTION_IDENTIFIER = 2
		self.HOME_CONFIG = [0,15,180,230,0,55,90]
		self.config = [0,15,180,230,0,55,90]
		
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
		
		
	def main(self):
	
		#*******************************************************************************
		# Make sure to clear the robot's faults else it won't move if it's already in fault
		self.clear_faults()
		#*******************************************************************************

		#*******************************************************************************
		# Move the robot to the Home position with an Action
		self.home_the_robot(sleep_time=5)
		#*******************************************************************************
		
		#*******************************************************************************
		# Set the reference frame to "Mixed"
		self.set_cartesian_reference_frame()
		#*******************************************************************************		
		
		#*******************************************************************************
		# Cartesian space movement demo
		self.send_cartesian_pose([0,0,0.1], [0,0,0],sleep_time=3)
		self.send_cartesian_pose([0,0,-0.2], [0,0,0],sleep_time=3)
		self.home_the_robot(sleep_time=3)
		
		self.send_cartesian_pose([0.1,0,0], [0,0,0],sleep_time=3)
		self.send_cartesian_pose([-0.2,0,0], [0,0,0],sleep_time=3)
		self.home_the_robot(sleep_time=3)
		
		self.send_cartesian_pose([0,0,0], [90,0,0], speed_orient=45, sleep_time=23)
		self.home_the_robot(sleep_time=10)
		
		self.send_cartesian_pose([0,0,0], [-30,0,0], speed_orient=45, sleep_time=8)
		self.home_the_robot(sleep_time=5)
		#*******************************************************************************
		
		#*******************************************************************************
		# Cartesian space movement demo, controlled by absulote pose
		self.send_cartesian_pose_abs([0.5,0,0.4],[90,0,180],sleep_time=20)
		#*******************************************************************************
		
		#*******************************************************************************
		# Joint space movement demo
		# Rotate the manipulator by 45 degree
		config = self.HOME_CONFIG
		config[0] = config[0]+45
		self.send_joint_angles_abs(config)
		
		self.send_joint_angles_rot([-15,0,0,0,0,0,0],sleep_time=3)
		#*******************************************************************************
		
		#*******************************************************************************
		# Gripper demo
		# rospy.loginfo("Gripper present status... %s", self.is_gripper_present)
		self.send_gripper_status(0.5)
		self.send_gripper_status(1)
		#*******************************************************************************
		
		self.home_the_robot(init_gripper=True)
		
				
		
if __name__ == "__main__":
	ex = ArmControllerDemo()
	ex.main()
	


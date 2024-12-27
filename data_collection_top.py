#! /usr/bin/env python

import os
import shlex
import subprocess
import sys
import time
from datetime import datetime

import json
import numpy as np
import cv2
import moveit_commander
import pathlib2 as pathlib
import rospy
import sensor_msgs.msg
import franka_msgs.msg
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, SwitchController, SwitchControllerRequest
from cv_bridge import CvBridge
from franka_control.msg import ErrorRecoveryActionGoal


class StrawberryDataCollection: 
	def __init__(self, data_dir, velocity_scaling_factor = 0.15, read_depth = False, read_pc = False, starting_sample = 0, record_topics = None,
	 current_configuration = 1, current_strawberry = 4):
		print("-------------------------------")
		"""Strawberry data Collection.

		Args:
			data_dir (str): Directory where all outputs will be saved.
			velocity_scaling_factor (float, optional): Robot velocity scaling factor. Defaults to 0.15.
			read_depth (bool, optional): If True, save depth images. Defaults to False.
			read_pc (bool, optional): If True, save pointclouds. Defaults to False.
			starting_experiment (int, optional): Index of the starting experiment. Defaults to 0.
			starting_sample (int, optional): Index of the starting sample. Defaults to 0.
			record_topics (list[str], optional): List of topics names to record. Defaults to ["/tf", "/joint_states"].
		"""

		# Settings.
		self.DATA_DIR = data_dir
		self.VELOCITY_SCALING_FACTOR = velocity_scaling_factor
		self.READ_DEPTH_IMAGE = read_depth
		self.READ_POINTCLOUD = read_pc
		self.RECORD_TOPICS = record_topics or ["/tf", "/joint_states"]
		self.current_sample = starting_sample
		self.current_configuration = current_configuration
		self.current_strawberry = current_strawberry

		# Other attributes.
		self.bridge = CvBridge()
		self.rosbag_proc = None
		self.current_controller = None
		self.show_img = False
		self.color_img = None
		self.depth_img = None
		self.ee_state_full = []
		self.joint_full = []
		self.captured_image = False

		# Robot initialization.
		NODE_NAME = "scene_data_collection"
		rospy.init_node(NODE_NAME)
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		GROUP_NAME = "panda_arm"
		self.group = moveit_commander.MoveGroupCommander(GROUP_NAME)
		self.group.set_max_velocity_scaling_factor(self.VELOCITY_SCALING_FACTOR)

		# Setup subscriptions.
		self.img_sub_top = rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, self.color_img_cb_top)
		self.recoverError = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=10)
	

		# Names and IDs of the available robot controllers.
		self.CONTROLLER_NAME2ID = {
			"torque": "franka_zero_torque_controller",
			"position": "position_joint_trajectory_controller",
			"impedance": "panda_leader_cartesian_impedance_controller",
			"impedance_advanced": "cartesian_impedance_advanced_controller"
		}
		# IDs and names of the available robot controllers.
		self.CONTROLLERS_ID2NAME = {c_id: c_name for c_name, c_id in self.CONTROLLER_NAME2ID.items()}

		# Get the name of the currently running robot controller.
		running_controllers = self.get_controllers_id(filter_state="running")
		if running_controllers:
			self.current_controller = running_controllers[0]
		else:
			self.current_controller = None
			
		# Create output directories.
		pathlib.Path(self.experiment_path).mkdir(parents=True, exist_ok=True)

	def color_img_cb_top(self, msg):
		"""Callback to get the RGB image from the ROS message."""

		# Get image from ROS message.
		color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		# Convert from BGR to RGB.
		self.color_img_top = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

		self.show_img = True

	@property
	def experiment_path(self):
		"""Full path of the current experiment."""
		return os.path.join(self.DATA_DIR, "scene" + str(self.current_configuration))

	def go_to_joint_state(self):

		self.enable_controller("position")

		# Home goal position
		joint_goal = [-0.015823606483903702, -0.5532147839541621, 0.046813822646935774, -2.2140220930323675, 0.011336741028532622, 1.6957928532886886, 0.870879934824175]
		# High camera position
		# joint_goal = [0.040290338347739416, -0.4087265696734712, -0.06278346424528136, -1.3853470862777753, -0.006666642369668655, 1.0616063620193217, 0.813244813865663]	
		self.group.go(joint_goal, wait=True)
		self.group.stop() # ensures there are no residual movements

		self.enable_controller("impedance")

	def move_away_from_cam_view(self):

		self.enable_controller("position")

		# robot_move_away position
		joint_goal = [1.0919453286394323, -0.5455106949053311, 0.412830394568123, -2.1955548851689404, 0.27329170688907606, 1.662487891197229, 0.49492896894282756]
		self.group.go(joint_goal, wait=True)
		self.group.stop() # ensures there are no residual movements

		self.enable_controller("impedance")


	def get_controllers_id(self, filter_state=None):
		"""Get a list of the IDs of the avilable controllers.

		Args:
			filter_state (str, optional): If specified, only return controllers with matching state. Defaults to None.

		Returns:
			list[str]: List of the IDs of the controllers.
		"""

		rospy.wait_for_service("/controller_manager/list_controllers")
		try:
			request = ListControllersRequest()
			service = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
			response = service(request)
		except Exception as e:
			rospy.logerr("List controllers server is down. Unable to list running controllers.")
			rospy.logerr(e)
			return False

		if filter_state is None:
			return [c.name for c in response.controller]
		else:
			return [c.name for c in response.controller if c.state == filter_state]


	def enable_controller(self, controller_name):
		"""Enable a certain controller for the robot.

		Args:
			controller_name (str): The name of the controller. Valid options are "torque" or "position".

		Returns:
			bool: Success.
		"""


		if self.current_controller == controller_name:
			rospy.loginfo("Controller '{}' is already active.".format(self.current_controller))
			return True

		if controller_name not in self.CONTROLLER_NAME2ID.keys():
			rospy.logerr("Controller '{}' is not a valid controller! Not switching.".format(controller_name))
			return False

		running_controllers_id = self.get_controllers_id("running")
		# Limit to controllers specified in self.CONTROLLERS. This excludes the state controller, which should always be running.
		running_controllers_id = [c_id for c_id in running_controllers_id if c_id in self.CONTROLLER_NAME2ID.values()]

		rospy.wait_for_service("/controller_manager/switch_controller")
		try:
			request = SwitchControllerRequest()
			request.strictness = 1
			# Stop all running controllers.
			request.stop_controllers = running_controllers_id
			# Start the required controller.
			request.start_controllers = [self.CONTROLLER_NAME2ID[controller_name]]
			service = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
			response = service(request)
		except Exception as e:
			rospy.logerr("Switch controller server is down. Unable to switch contoller to '{}'".format(controller_name))
			rospy.logerr(e)
			return False

		if not response.ok:
			rospy.logerr("Failed to switch to controller '{}'".format(controller_name))
			return False

		self.current_controller = controller_name
		
	def capture_images(self, otuput_dir, sample_id):
		"""Capture RGB, depth and/or pointcloud from the current position.
			sample_id (str): Sample id for the current capture.
		"""
		# if self.moved_away == True:
		# RGB.
		img_path_top = os.path.join(otuput_dir, "rgb_img_" + sample_id + ".png")

		cv2.imwrite(img_path_top, self.color_img_top)

		rospy.loginfo("Recorder image '{}'".format(self.current_sample))


	def record_bag_file(self):
		"""Record a ROS bag."""


		save_path = os.path.join(self.experiment_path,
								"scene" + str(self.current_configuration) + "_object" + str(
									self.current_strawberry) + "_traj" + str(self.current_sample))
		# Ensure that the output directory exists. 
		pathlib.Path(self.experiment_path).mkdir(parents=True, exist_ok=True)

		# Record the ROS bag.
		rospy.loginfo("Recording bag to '{}'.".format(save_path))
		# TODO: Implement in rospy (Is it possible to have it run parallely that way?).
		command = "rosrun rosbag record " + " ".join(self.RECORD_TOPICS) + " -O '{}'".format(save_path)
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		# Save in a numpy file
		# ee_state = self.group.get_current_pose().pose
		# ee_state = self.ee_state_full.append([ee_state.position.x, ee_state.position.y, ee_state.position.z, ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])
		#print(ee_state)
		#data = np.asarray(ee_state)
		#np.save(save_path, data)

	
	def stop_recording(self):
		"""Stop recording a ROS bag.
		
		Returns:
			bool: Success.
		"""

		if self.rosbag_proc is None:
			rospy.logwarn("No recording in progress: you can start one by pressing 's'.")
			return False

		# Stop recording.
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc = None
		rospy.loginfo("Finished recording.")

		
		return True

	def next_strawberry(self):
		"""Move to the next strawberry."""

		# Set indices.
		self.current_strawberry += 1
		self.current_sample = 0

		# Ensure output directories exist.
		pathlib.Path(self.experiment_path).mkdir(parents=True, exist_ok=True)

		rospy.loginfo("Starting experiment {}".format(self.current_strawberry))


	def recover_error(self):
		"""Quickly recover from a soft error (e.g. someone pushed the robot while in position control."""

		command = ErrorRecoveryActionGoal()
		self.recoverError.publish(command)


	@property
	def usage(self):
		return """
Press any of these keys to execute the corresponding command:

[Enter]  Show this help message.
z    Close and exit.
s        Start collecting a sample.
f        Finish collecting a sample.
n        Start for the next object.
i  Collect images from the camera.
p        Enable position control.
t        Enable zero-torque control.
e        Try to recover the robot from a soft error.
h        Go to the home position.
m	Go to sideways position
"""
	
	def loop(self):
		"""Main event loop."""
		
		
		self.recover_error()
		# data_collection = StrawberryDataCollection(data_dir="/home/general_vlm/dataset_collection/src/dataset")
		# Setup.
		self.enable_controller("position")
		pathlib.Path(self.experiment_path).mkdir(parents=True, exist_ok=True)

		rospy.loginfo("Starting the main event loop.")
		rospy.loginfo("Press [Enter] to show the help text.")

		# temp = -1

		while not rospy.is_shutdown():

			save_path = os.path.join(self.experiment_path,
								 "scene" + str(self.current_configuration) + "_object" + str(
									 self.current_strawberry) + "_traj" + str(self.current_sample))

			save_path_joint = os.path.join(self.experiment_path,
								 "scene" + str(self.current_configuration) + "_object" + str(
									 self.current_strawberry) + "_traj" + str(self.current_sample) + "_joints")
			# Display RGB image.
			image = np.zeros(  ( 200, 200 ) , dtype = np.uint8)
			cv2.imshow( "Switch Control", image ) 
			k = cv2.waitKey(30)
			# if not k == temp:
			# 	print(k)
			# 	temp = k

			




			# Command initialize.
			esc = "esc"
			legend = "legend"
			h = "h"
			i = "i"
			e = "e"
			t = "t"
			p = "p"
			n = "n"
			b = "b"
			s = "s"
			f = "f"
			m = "m"
			recording = False

			# Taking input from keyboard

			if k == "z" or k == int(122):  # Esc
				"""Exit."""
				cv2.destroyAllWindows()
				return

			elif k == "l" or k == int(108) or k == int(13):	# [Enter] or "l"
				"""Display the help text."""
				print(self.usage)

			elif k == h or k == int(104):
				"""Go to home position."""

				self.go_to_joint_state()

			elif k == m or k == int(109):
				"""Go to the sideways position"""
				print("Moving to the side")

				self.move_away_from_cam_view()

			elif k == i or k == int(105):
				"""Capture an image."""

				self.current_sample = self.current_sample + 1

				self.capture_images(self.experiment_path, "scene_"+str(self.current_configuration)+
									"_object_"+str(self.current_strawberry)+ "_traj_"+str(self.current_sample))

				

			elif k == "e" or k == int(101):
				"""Recover from an error."""

				self.recover_error()

			elif k == "t" or k == int(116):
				"""Enable impedance controller."""

				self.enable_controller("impedance")

			elif k == "p" or k == int(112):
				"""Enable position controller."""
				self.recover_error()

				self.enable_controller("position")

			elif k == n or k == int(110):
				"""Go to the next object."""
				print("Next object")

				self.next_strawberry()



			elif k == s or k == int(115):
				"""Start sample collection."""
				recording = True
				self.ee_state_full = []
				self.joint_full = []
				self.start_time = time.time()



				if self.rosbag_proc is not None:
					rospy.logerr("Another recording is already in progress!")
					rospy.logerr("Stop it with 'f' to continue.")
					return

				# self.capture_images(self.experiment_path, "scene"+str(self.current_configuration)+
				# 					"_object"+str(self.current_strawberry)+"_traj"+str(self.current_sample))

				# Enable torque control.
				self.enable_controller("impedance")    # Switch to zero-torque control to move the roboy by hand
				self.record_bag_file()
				# Prepare for next sample.
				# self.current_sample += 1

			elif k == f or k == int(102):
				"""Finish sample collection."""
				recording = False

				SLEEP_TIME = 1.0  # [s]

				if self.stop_recording():
					self.enable_controller("position")      # switch back to position control to move to home position
					time.sleep(SLEEP_TIME)
				

			while recording:
				#print("\n\n", len(self.ee_state_full))
				ee_state = self.group.get_current_pose().pose
				joint = self.group.get_current_joint_values()
				self.ee_state_full.append([ee_state.position.x, ee_state.position.y, ee_state.position.z, ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])
				self.joint_full.append([joint])
				#print(ee_state)
				key = cv2.waitKey(30)
				if (key == f or key == int(102)):
					self.end_time = time.time()
					print(self.end_time)
					np.save(save_path, np.array(self.ee_state_full))
					np.save(save_path_joint, np.array(self.joint_full))
					recording = False
					rospy.loginfo("Press f again to stop recording and go back to home position")
					#print("rate = ", float(len(self.ee_state_full)) / (self.end_time - self.start_time))
					#print(self.start_time, self.end_time)
					#print(self.end_time - self.start_time)



if __name__ == '__main__':
	data_collection = StrawberryDataCollection(data_dir="/home/general_vlm/dataset_collection/src/dataset")
	data_collection.loop()
	

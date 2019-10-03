#!/usr/bin/env python

#Run roslaunch ur3_driver ur3_driver.launch in the first terminal
#Be sure to source devel/setup.bash in the second terminal
#Run rosrun lab2pkg_py lab2_exec.py at least once or you might not see some data.
import sys
import copy
import time
import rospy
import numpy as np
from lab2_header import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = np.radians([147.3, -101.9, 135.5, -123.8, -89.9, 70])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


############## Your Code Start Here ##############
# Hanoi tower locations

Q11 = np.radians([147.9, -52.2, 114.1, -152.8, -90, 57])
Q12 = np.radians([147.9, -58.71, 113.4, -145.7, -90, 57])
Q13 = np.radians([147.9, -65.3, 111.3, -137.1, -90, 57])

Q21 = np.radians([160.2, -53.5, 116.82, -154.4, -90, 70])
Q22 = np.radians([160.2, -60.3, 116.1, -146.8, -90, 70])
Q23 = np.radians([160.2, -66.8, 114.1, -138.4, -90, 70])

Q31 = np.radians([173.2, -52.4, 114.9, -153.5, -90, 82.3])
Q32 = np.radians([173.2, -59.4, 114.11, -145.7, -90, 82.3])
Q33 = np.radians([173.2, -65.6, 112.2, -137.6, -90, 82.3])

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]

############### Your Code End Here ###############
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
	global digital_in_0
	global analog_in_0

	digital_in_0 = msg.DIGIN & 1
	analog_in_0 = msg.AIN0


############### Your Code End Here ###############
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0   
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


############## Your Code Start Here ##############
def move_block(pub_cmd, loop_rate, start_loc, start_height, \
	           end_loc, end_height):
	global Q
	error = 0

	return error

############### Your Code End Here ###############
def main():

	global home
	global Q
	global SPIN_RATE

	# Initialize ROS node
	rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)

	############## Your Code Start Here ##############
	# TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

	rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

	############### Your Code End Here ###############


	############## Your Code Start Here ##############
	# TODO: modify the code below so that program can get user input

	input_done = 0

	while(not input_done):
		input_string = raw_input("Enter start location:")
		print("You entered " + input_string + "\n")

		if(int(input_string) == 1):
			input_done = 1
			start_location = 1
		elif (int(input_string) == 2):
			input_done = 1
			start_location = 2
		elif (int(input_string) == 3):
			input_done = 1
			start_location = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")

	input_done = 0
	while(not input_done):
		input_string = raw_input("Enter End location:")
		print("You entered " + input_string + "\n")

		if(int(input_string) == 1):
			input_done = 1
			end_location = 1
		elif (int(input_string) == 2):
			input_done = 1
			end_location = 2
		elif (int(input_string) == 3):
			input_done = 1
			end_location = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")

	inter_ind = 6 - start_location - end_location - 1
	start_ind = start_location - 1
	end_ind = end_location - 1


	############### Your Code End Here ###############
	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	rospy.loginfo("Sending Goals ...")

	loop_rate = rospy.Rate(SPIN_RATE)

	############## Your Code Start Here ##############
	# TODO: modify the code so that UR3 can move tower accordingly from user input
	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#FIRST BLOCK MOVE

	rospy.loginfo("Moving First Block")
	move_arm(pub_command, loop_rate, Q[start_ind][2], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[end_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#SECOND BLOCK MOVE

	rospy.loginfo("Moving Second Block")
	move_arm(pub_command, loop_rate, Q[start_ind][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[inter_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#THIRD BLOCK MOVE

	rospy.loginfo("Moving Third Block")
	move_arm(pub_command, loop_rate, Q[end_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[inter_ind][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#FOURTH BLOCK MOVE

	rospy.loginfo("Moving Fourth Block")
	move_arm(pub_command, loop_rate, Q[start_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[end_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#Fifth BLOCK MOVE

	rospy.loginfo("Moving Fifth Block")
	move_arm(pub_command, loop_rate, Q[inter_ind][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[start_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#Sixth BLOCK MOVE

	rospy.loginfo("Moving Sixth Block")
	move_arm(pub_command, loop_rate, Q[inter_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[end_ind][1], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############################################################################
	############################################################################
	#Seventh BLOCK MOVE

	rospy.loginfo("Moving Seventh Block")
	move_arm(pub_command, loop_rate, Q[start_ind][0], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_on)

	# Delay to make sure suction cup has grasped the block
	time.sleep(1.0)
	if digital_in_0 == 0:
		rospy.loginfo("Error: Block Not Present")
		gripper(pub_command, loop_rate, suction_off)
		move_arm(pub_command, loop_rate, home, 4.0, 4.0)
		sys.exit()
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	move_arm(pub_command, loop_rate, Q[end_ind][2], 4.0, 4.0)
	gripper(pub_command, loop_rate, suction_off)

	rospy.loginfo("Moving Home")
	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	############### Your Code End Here ###############

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
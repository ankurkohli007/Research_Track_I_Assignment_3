#!/usr/bin/python3

"""
This is a master_control_node which will accepts and reads the user request and change the behaviour of the robot acording to user input.
"""

# importig required libraries
from std_srvs.srv import *
import math
import rospy



 

msg = """
**Robot's Behaviour options**
#Press 1: Autonomously driving robot to reach at x,y coordinates as per input by the user 
#Press 2: Teleop operations by the user drive the robot using keyboard
#Press 3: Obstacle avoidance operations to drive the robot assisting them (using keyboard) to avoid collisions 
#Press 0: Lazy state of robot, in this state robot will shows laziness and do nothing untill and unless the robots's behvaiour is chnaged by the user

anything else : stop

q/z : accelerate/decelrate velocity by 10%
w/x : accelerate/decelrate only linear velocity by 10%
e/c : accelerate/decelrate only angular velocity by 10%

CTRL-C to quit
"""

#main() is the the point of execution to perform operations
def main():
	"""
	In main(), user will be asked contantly for the input to change the robot's behaviour such as autonomously drive mode, teleop
	keyboard mode, obstacle avoidance mode, idle mode, and so on. 
	"""
	
	f = False	# this will print a goal cancellation message whenever any operation under execution
	while not rospy.is_shutdown(): 
		command = int(input('Benvenuti!! Please choose the operation to change robot behaviour'))	
		# Now checking the option entered by the user to change robot state
							
		# OPERATION 1: AUTONOMOUS DRIVING MODE TO REACH AT X,Y COORDINATES (User's Desired Position)
		if command == 1:
		
			if f == True:
			     print("Sorry!! Cancelling the operation")
			     f = 0
				
			rospy.set_param('active', 0) # resetting robot's state
			
			# Displaying message for the user to know the actuale state of robot 
			print("Now robot is in operation 1 state, if you want to cancel robot's behaviour then press '0'!!")
			# Now asking the user to input the target position i.e. x,y coordinated where user want to move the robot
			print("Please, enter the target position (x,y coordinates) where you want to send the robot!!")
			pos_x = float(input("Please, enter X coordinate:"))	# Displaying message to input x coordinate
			pos_y = float(input("Please, enter Y coordinate: "))   # Displaying message to input y coordinate		
			
			# Setting parameter on the Parameter Server
			rospy.set_param('des_pos_x', pos_x)	
			rospy.set_param('des_pos_y', pos_y)	
			rospy.set_param('active', 1)		
			print("ROBOT is performing opertion 1")
			f = True
			
		# OPERATION 2: OBSTACLE OPERATION TO CHANGE ROBOT'S BEHAVIOUR USING KEYBOARD
		elif command == 2:
		

			if f == True:
			     print("Sorry!! Cancelling the operation")
			     f = False
						
			rospy.set_param('active', 2)   # Message to know robot's current state
			print("Now robot is performing Operation 2")
			
				
		# OPERATION 3: OBSTACLE OPERATION TO ASSIST ROBOT'S BEHAVIOUR USING KEYBOARD & AVOIDING COLLISION
		elif command == 3:
		
			if f == True:
			     print("Sorry!! Cancelling the operation")
			     f = False
				
			rospy.set_param('active', 3) 	# Message to know robot's current state
			print("Now robot is performing Operation 3")
				
		# OPERATION 4: LAZY STATE, ROBOT WILL NOT PERFORM ANY ACTION
		elif command == 0:
			
			if f == True:
			     print("Sorry!! Cancelling the operation")
			     f = False
				
			rospy.set_param('active', 0)   # Message to know robot's current state
			print("Now robot is performing Operation 4")	# Printing the actual state.
				
		else:
			
			print("Sorry!! Invalid input!! Please choose the coorect option to perform robot operations!")

# this is used to execute some code only if the file was run directly, and not imported.
if __name__ == '__main__':
	print(msg)
	main()



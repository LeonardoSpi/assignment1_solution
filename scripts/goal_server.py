#!/usr/bin/env python

import rospy
import random # This library is needed to computed random numbers

from assignment1_solution.srv import goal, goalResponse # Import custom service of type goal

def random_number_generator(): # This function returns a random number between -6 and 6
	n = random.uniform(-6,6)
	return n

def handle_goal_generator(req): # Callback of the service
	if(req.request is True): # The request sent by the client is a bool
				 # If it is True than the client wants a random goal

		x = random_number_generator() # Calls the function to generate coordinate x
		y = random_number_generator() # Calls the function to generate coordinate y

		print("Goal generated:    x = %f | y = %f" %(x,y))

		return goalResponse(x, y) # Returns the goal generated

	else: # If the bool request is False the clients wants to go back to the base

		x = 0 # Gives the coordinate x of the base (In this case 0)
		y = 0 # Gives the coordinate y of the base (In this case 0)

		print("Returning to base: x = %f | y = %f" %(x,y))

		return goalResponse(x, y) # Returns the goal generated
		

def goal_server():
	rospy.init_node('goal_server') # Initialize the node as goal_server
	s = rospy.Service('goal_generator',goal, handle_goal_generator) # The nodes works as a server for the
								 	# service goal_generator

	print("Ready to generate goal:") # Once the node is started, print to screen so that
					 # the user knows it is ready to generate a goal

	rospy.spin() # Continue to cycle

if __name__ == '__main__':
    try:
        goal_server() # Execute function goal_server()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        pass

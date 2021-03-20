#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry # Import the message type needed for position
from geometry_msgs.msg import Twist # Import the message type needed for velocities
from assignment1_solution.srv import goal, goalResponse # Import custom service
import math # Needed for pi and cos

vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1000) # Define the node as a publisher on the topic 								    # /cmd_vel with a message of 
							    # type geometry_msgs/Twist

client = rospy.ServiceProxy('/goal_generator', goal) #Define the node as a client for the service
						     #goal_generator with the custom service goal

resp = client(1) # Calls for service goal_generator giving 1 as bool request, the server will respond
		 # generating a random goal and giving x and y coordinates to be reached

def velocity(x, y, t, tx, ty): # Velocity function takes as arguments:
			       # x  = the distance in x
			       # y  = the distance in y
			       # t  = the starting time
			       # tx = period of function vx
			       # ty = period of function vy

	print("Computing velocities")

	now = rospy.get_time() # Get the time now and store it in the variable "now"

	print(now-t)
 
	print("Period of x is : %f" %(tx))
	print("Period of y is : %f" %(ty))

	vel_msg = Twist() # vel_msg is defined as a message of type Twist()

	if((now-t)/(tx)>1): # Check if we are inside the period of function vx

		vx = 0 # The velocity should be null if 1 period has passed
	else:
		vx = ((x)/(abs(x))*0.5)*(1 - math.cos(2*math.pi*(now-t)/(tx))) # Velocity function for x

	if((now-t)/(ty)>1): # Check if we are inside the period of function vy

		vy = 0 # The velocity should be null if 1 period has passed
	else:
		vy = ((y)/(abs(y))*0.5)*(1 - math.cos(2*math.pi*(now-t)/(ty))) # Velocity function for y

	vel_msg.linear.x = vx # Store the velocity vx inside vel_msg.linear.x
	vel_msg.linear.y = vy # Store the velocity vy inside vel_msg.linear.y

	vel_pub.publish(vel_msg) # Publish the velocities in the /cmd_vel topic

	print("Velocities are x: %f, y: %f" %(vx, vy))
	
def control():
	rospy.init_node('robot_controller') # Initialize the node with the name robot_controller

	rate = rospy.Rate(100) # Rate of the spinning loop will be 100hz

	position = rospy.wait_for_message("/odom", Odometry) # Wait for 1 message to be publish on topic /odom

	distance_x = resp.x - position.pose.pose.position.x # Compute distances between the goal and the
	distance_y = resp.y - position.pose.pose.position.y # current position of the robot

	period_x = abs(2*distance_x) # Since the velocities in x and y will be computed with specific 
	period_y = abs(2*distance_y) # law of motions, we need to compute periods for both
				     # which depend on the distance

	print("Goal generated is x: %f, y: %f" %(resp.x, resp.y))     # Print informations about goal coordinates
	print("Distances are x: %f, y: %f" %(distance_x, distance_y)) # and distances

	t = rospy.get_time() # Get the time and save it in the variable t

	while(((abs(distance_x))>0.05) or ((abs(distance_y))>0.05)): # Robot should move until the distances from
								     # goals go under 0.05

		velocity(distance_x, distance_y, t, period_x, period_y) # Execute function velocity
									# passing the distances,
									# the time saved in t and the periods
		position = rospy.wait_for_message("/odom", Odometry) # Wait for 1 message to be publish on 
								     #topic /odom

		distance_x = resp.x - position.pose.pose.position.x # Update current distance in x
		distance_y = resp.y - position.pose.pose.position.y # Update current distance in x
	
		print("Distances are x: %f, y: %f" %(distance_x, distance_y)) # Print current distances

	print("Goal reached") # When the distances go under 0.05 than we exit from the while loop
			      # so we can print "goal reached" and the final position acquired

	print("Position is x: %f, y: %f" %(position.pose.pose.position.x, position.pose.pose.position.y))
	
	
if __name__ == '__main__':
    try:
        control() # Execute function control()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        pass

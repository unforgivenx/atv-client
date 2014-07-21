#!/usr/bin/env python
# A python program to control ATV via x-box joypad
# By: Yaser Mohamadi
# INMATE lab, Middle East Technical University
   

import rospy	# To make the program act as a ROS node
import pygame	# To use Joystick Module
from std_msgs.msg import Float32, Int16, Int16MultiArray, MultiArrayDimension

pub = rospy.Publisher('joy', Float32)			# Publishes joystick's raw data to 'joy' topic
steer = rospy.Publisher('steering_perc', Int16)		# Publishes steering message to 'steering_perc' topic
throt = rospy.Publisher('throttle', Int16MultiArray)	# Publishes throttle message to 'throttle' topic
brake = rospy.Publisher('brake_perc', Int16)		# Publishes braking message to 'brake_perc' topic

# Throttle standard messages:
backward = Int16MultiArray()
backward.layout.dim = [MultiArrayDimension('data', 1, 4)]
backward.data = (1,75,1,75)		# Direct Backward
backward_stop = Int16MultiArray()
backward_stop.layout.dim = [MultiArrayDimension('data', 1, 4)]
backward_stop.data = (1,0,1,0)		# Backward to Still
forward_stop = Int16MultiArray()
forward_stop.layout.dim = [MultiArrayDimension('data', 1, 4)]
forward_stop.data = (0,0,0,0)		# Forward to Still
forward = Int16MultiArray()
forward.layout.dim = [MultiArrayDimension('data', 1, 4)]
forward.data = (0,75,0,75)		# Direct Forward
right = Int16MultiArray()
right.layout.dim = [MultiArrayDimension('data', 1, 4)]
right.data = (0,75,0,0)		# Right Turn
left = Int16MultiArray()
left.layout.dim = [MultiArrayDimension('data', 1, 4)]
left.data = (0,0,0,75)		# Left Turn
right_hard = Int16MultiArray()
right_hard.layout.dim = [MultiArrayDimension('data', 1, 4)]
right_hard.data = (0,75,1,75)		# Hard Right Turn
left_hard = Int16MultiArray()
left_hard.layout.dim = [MultiArrayDimension('data', 1, 4)]
left_hard.data = (1,75,0,75)		# Hard Right Turn

rospy.init_node('joy', anonymous=True)		# Initiate ROS node 'joy'
r = rospy.Rate(5) 				# Frequency: 10hz

def joy():
	# Define some colors
	BLACK    = (   0,   0,   0)
	WHITE    = ( 255, 255, 255)
	GREEN    = (   0, 255,   0)
	RED      = ( 255,   0,   0)
	CYAN = (0,204,204)
	PINK = (255,102,178)
	PURPLE = (161,83,213)

	l=510	# Length of the screen
	w=510	# Width of the screen
	t=0.3	# Tolerance specified for safe zone

	# Paint the screen
	def draw_stick_figure(screen, x, y):
	    pygame.draw.rect(screen, GREEN, [204,204,102,102], 0)	# safe zone (for joystick tolerance)		
	    pygame.draw.rect(screen, CYAN, [0,204,204,102], 0)		# still turn (left)
	    pygame.draw.rect(screen, CYAN, [306,204,204,102], 0)	# still turn (right)
	    pygame.draw.rect(screen, PINK, [204,0,102,204], 0)		# direct forward
	    pygame.draw.rect(screen, PINK, [204,306,102,204], 0)	# direct bakward
	    pygame.draw.rect(screen, PURPLE, [0,0,204,204], 0)		# compound motion
	    pygame.draw.rect(screen, PURPLE, [306,0,204,204], 0)	# compound motion
	    pygame.draw.rect(screen, PURPLE, [0,306,204,204], 0)	# compound motion
	    pygame.draw.rect(screen, PURPLE, [306,306,204,204], 0)	# compound motion

	    pygame.draw.ellipse(screen, BLACK, [1 + x, y, 10, 10], 0)	# joystick pointer


	pygame.init()		# Initiate 'pygame'

	# Set the width and height of the screen [width,height]
	size = [l, w]
	# Start the screen
	screen = pygame.display.set_mode(size)

	pygame.display.set_caption("ATV via Joystick (Yaser & Kemal)")

 	#Loop until the user clicks the close button.
	done = False

	# Used to manage how fast the screen updates
	clock = pygame.time.Clock()

	# Count the joysticks the computer has
	joystick_count = pygame.joystick.get_count()
	if joystick_count == 0:
	    # No joysticks!
	    brake.publish(100) # Emergency Brake
	    print ("Error, I didn't find any joysticks.")
	else:
	    # Use joystick #0 and initialize it
	    my_joystick = pygame.joystick.Joystick(0)
	    my_joystick.init()

	while not done:

	    # Event processing
	    for event in pygame.event.get():
		if event.type == pygame.QUIT:
		    done = True

	    # Main stuff starts here!

	    if joystick_count != 0:	# As long as there is a joystick

		# This gets the position of the axis on the game controller
		# It returns a number between -1.0 and +1.0
		horiz_axis_pos = my_joystick.get_axis(0)	# Horizontal axis
		vert_axis_pos = my_joystick.get_axis(1)		# Vertical axis
#		brake_axis_pos = my_joystick.get_axis(4)	# Axis defined for braking    
		direct_steer_button = my_joystick.get_button(9)
#		LB = my_joystick.get_button(4)
#		RB = my_joystick.get_button(5)
#		LT = my_joystick.get_axis(2)
#		RT = my_joystick.get_axis(5)

		# Update the coordination of the joystick's pointer on the screen
		x_coord = (horiz_axis_pos*250)+500/2	# Puts the pointer in the middle (250 in the case of size 500) when reads 0
		y_coord = (vert_axis_pos*250)+500/2	# Puts the pointer in the middle (250 in the case of size 500) when reads 0

		# Braking:
#		if (my_joystick.get_button(1) == True):	# Full brake via braking button
#			brake.publish(120)	
#		else:					# Controlled brake via braking pedal
#			brake.publish(int(60 + (brake_axis_pos*120)/2))
#
		# Throttling & Steering actions: 

		if (-t<vert_axis_pos<0.0):		# t (out of 1.0) is chosen to safe zone's range
			throt.publish(forward_stop)		# Still
			r.sleep()
			if (-t<horiz_axis_pos<t):
				steer.publish(50)	# 'steer_perc' topic gets integer message in the range 0 - 100. 0>>-30deg, 50>>0deg, and 100>>30deg
			elif (horiz_axis_pos < -t) :
				steer.publish(int(50 + ((horiz_axis_pos+t)*50)/(1-t)))	# Mapping joystick data (-1 to 1) to steer_perc (0 to 100)
			elif (horiz_axis_pos > t) :
				steer.publish(int(51 + ((horiz_axis_pos-t)*50)/(1-t)))
		if (0.0<vert_axis_pos<t):		# t (out of 1.0) is chosen to safe zone's range
			throt.publish(backward_stop)		# Still
			r.sleep()
			if (-t<horiz_axis_pos<t):
				steer.publish(50)	# 'steer_perc' topic gets integer message in the range 0 - 100. 0>>-30deg, 50>>0deg, and 100>>30deg
			elif (horiz_axis_pos < -t) :
				steer.publish(int(50 + ((horiz_axis_pos+t)*50)/(1-t)))	# Mapping joystick data (-1 to 1) to steer_perc (0 to 100)
			elif (horiz_axis_pos > t) :
				steer.publish(int(51 + ((horiz_axis_pos-t)*50)/(1-t)))
		elif (vert_axis_pos < -t):
			throt.publish(forward)		# Forward
			r.sleep()
			if (-t<horiz_axis_pos<t):
				steer.publish(50)
			elif (horiz_axis_pos < -t) :
				steer.publish(int(50 + ((horiz_axis_pos+t)*50)/(1-t)))
			elif (horiz_axis_pos > t) :
				steer.publish(int(51 + ((horiz_axis_pos-t)*50)/(1-t)))
		elif (vert_axis_pos > t):
			throt.publish(backward)		# Backward
			r.sleep()
			if (-t<horiz_axis_pos<t):
				steer.publish(50)
			elif (horiz_axis_pos < -t) :
				steer.publish(int(50 + ((horiz_axis_pos+t)*50)/(1-t)))
			elif (horiz_axis_pos > t) :
				steer.publish(int(51 + ((horiz_axis_pos-t)*50)/(1-t)))



		# Some extra stuff for testing:
	 	#print horiz_axis_pos
		#print vert_axis_pos
		rospy.loginfo(y_coord)
		pub.publish(y_coord)
		r.sleep()
   
	    # Drawing stuff comes here:   
	    # First, clear the screen to WHITE. Don't put other drawing commands
	    # above this, or they will be erased with this command.
	    screen.fill(WHITE)    
	    # Draw the item at the proper coordinates
	    draw_stick_figure(screen, x_coord, y_coord)          
	    # Update the full display Surface to the screen
	    pygame.display.flip()
	    clock.tick(600)
pygame.quit()

if __name__ == '__main__':
	try:
		joy()
	except rospy.ROSInterruptException: pass

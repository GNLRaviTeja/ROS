#!/usr/bin/env python
import rospy, time
import argparse
import numpy as np
import os
import tf
import thread
import math
#import SignsDetection as SD 
import keras as kr
from keras.preprocessing import image
from keras.models import Sequential, load_model



# ROS Image message
from sensor_msgs.msg import Image
# ROS Laser message
from sensor_msgs.msg import LaserScan
# ROS Geometry/Velocity message
from geometry_msgs.msg import Twist, Pose, Quaternion
# ROS String message
from std_msgs.msg import String
# ROS Odometry(Pose/Position) message
from nav_msgs.msg import Odometry
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Instantiate CvBridge
bridge = CvBridge()

def vel_msg_init():
	vel_msg = Twist()
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	return vel_msg

def detect_signs(sign_image):
    kr.backend.clear_session()
    test_image = image.load_img(sign_image, target_size = (64, 64))
    test_image = image.img_to_array(test_image)
    test_image = np.expand_dims(test_image, axis = 0)
    classifier = load_model('model6.h5')
    pred = classifier.predict(test_image)
    result = pred[0]
    answer = np.argmax(result)
    if answer == 0:
        return("left")
    elif answer == 1:
        return("misc")
    elif answer == 2:
        return("right")
    elif answer == 3:
        return("stop")
    elif answer == 4:
        return("traffic_light")
    elif answer == 5:
        return("u")

## this function is called via thread to keep the processing out of the subscriber/callback functions
def threaded_image_handling(cv2_img):
	global turn, recognition_running
	print("Processing Image data")
	cv2.imwrite('camera_img.jpeg', cv2_img)	## save image
	##call your recognition model here and set the value of turn
	#turn = SD.detect_signs('camera_img.jpeg')
	turn = detect_signs('camera_img.jpeg')
        print(turn)
	print("after turn")
	#h=turn
	recognition_running = False

## this subscriber function is called every ~millisecond with the bot view image as msg
def image_callback(msg):
	# print("Received an image!")
	global recognition_running
	# if laser_running == False:
	# 	thread.start_new_thread(threaded_line_follower, (cv2_img,))
	
	if recognition_running == True:	
		# print("Waiting for previous recognition thread to finish processing")
		pass
	else:
		global turn		##accessing global variable turn
		if turn=="na":
			recognition_running = True
			try:
				# Convert your ROS Image message to OpenCV2
				cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
				recognition_running = False
			else:
				# print("Starting an image handling thread")
				thread.start_new_thread(threaded_image_handling, (cv2_img,))
				# Do image processing inside the thread
		else:
			## havent completed a detected turn yet, therefore no need to run the recognition pipeline
			# print("---------Waiting for", turn,"turn to be completed--------")
			pass
			 	
def threaded_deviation_handling():
	global orientation, curr_x, curr_y, pos_y, pos_x, active_x, active_y, deviation_running
	cur_x = curr_x
	cur_y = curr_y
	# print("X-Diff:",abs(pos_x-cur_x),"Y-Diff:")
	if active_x==True and abs(pos_x-cur_x)>dev_thr:	## check if there is deviation X values and correct them to pos_x
		print("Correcting X position")
		perp = linear_vel*1		## perp dist travelled = speed * time
		base = abs(pos_x-cur_x)	## base is deviation
		ang = math.atan(perp/base)	## get angular speed as angle
		if (cur_x<pos_x and orientation>0) or (cur_x>pos_x and orientation<0):
			sign = -1 	##move right
		else:
			sign = 1	##move left
		ang_vel = sign*ang
		vel_msg = vel_msg_init()
		vel_msg.linear.x = linear_vel
		vel_msg.angular.z = ang_vel/3
		velocity_publisher.publish(vel_msg)
		
	elif active_y==True and abs(pos_y-cur_y)>dev_thr:	## check if there is deviation Y values and correct them to pos_y
		print("Correcting Y position")
		perp = linear_vel*0.8
		base = abs(pos_y-cur_y)
		ang = math.atan(perp/base)
		if (cur_y<pos_y and (orientation>PI/2 or orientation<-1*PI/2)) or (cur_y>pos_y and (orientation<PI/2 and orientation>-1*PI/2)):
			sign = -1 	##move right
		else:
			sign = 1	##move left
		ang_vel = sign*ang*0.1
		vel_msg = vel_msg_init()
		vel_msg.linear.x = linear_vel
		vel_msg.angular.z = ang_vel
		velocity_publisher.publish(vel_msg)
		
	deviation_running=False


## this subscriber function is called every ~millisecond with the bot's orientation and pose
def odom_callback(msg):
	# print("position:",msg.pose.pose.position)
	# print("orientation:",msg.pose.pose.orientation)
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	## yaw relects orientation of the bot
	# print("yaw:",yaw,"orientation:",msg.pose.pose.orientation)
	global orientation, curr_x, curr_y, deviation_running
	curr_y = msg.pose.pose.position.y
	curr_x = msg.pose.pose.position.x
	orientation = yaw
	## the right turn may not be perfect and even a new orientation of 88 degrees will change the bot's course
	## Hence, we implement a small program to correct its path
	if laser_running == False and deviation_running==False:
		deviation_running = True
		thread.start_new_thread(threaded_deviation_handling,())
		## you can implement lane detection or lline follower in this thread


## this function is called via thread to keep the processing out of the subscriber/callback functions
def threaded_obstacle_handling():
	
	global turn, laser_running
	############################################################################################
	# print("!!!!!!!!!     Wall Detected! Stop bot for turn     !!!!!!!!")
	vel_msg = vel_msg_init()
	velocity_publisher.publish(vel_msg)
	# time.sleep(0.5)
	############################################################################################
	
	############################################################################################
	# print("!!!!!!!!!     Implement Obstacle Avoidance / TURN        !!!!!!!!!!")
	
	if turn == "na":
		# print("===========Implement Obstacle Avoidance here===========")	## no turn sign but barruer ahead
		##
		pass
	else:
		print("!!!!!!!!   TURNING BOT   !!!!!!!!")
		global orientation
		angular_speed = turn_ang_speed
		vel_msg = vel_msg_init()
		pos = np.array((PI, PI/2, 0, -PI/2, -PI))
		i = np.argmin(abs(pos-orientation))		## get which orientation is the closest
		if turn == "right":
			if i == 4:
				i=0
			target=pos[i+1]
			relative_angle = PI/2
			vel_msg.angular.z = -abs(angular_speed)	## clockwise
		elif turn == "left":
			if i == 0:
				i=4
			target=pos[i-1]
			relative_angle = PI/2
			vel_msg.angular.z = abs(angular_speed)	## anti-clockwise
		elif turn == "u":
			target=pos[(i+2)%4]
			relative_angle = PI
			vel_msg.angular.z = -abs(angular_speed)	## clockwise
		
		eps = degree_precision	## error of 0.5 degree is allowed
		while( abs(orientation - target) > eps):
			# print("orientation:", orientation,"target:",target)
			vel_msg = vel_msg_init()
			if orientation<target:
				vel_msg.angular.z = abs(angular_speed)
				# vel_msg.angular.z = max(10*PI/180,2*abs(orientation-target))
			else:
				vel_msg.angular.z = -abs(angular_speed)
				# vel_msg.angular.z = min(-10*PI/180,-2*abs(orientation-target))
			velocity_publisher.publish(vel_msg)

		############################################################################################
		############################################################################################
		# print("=== (((Obstacle Avoidance / TURN Completed))): Stabilize the Speed and Pose of bot ===")
		# vel_msg = vel_msg_init()
		# velocity_publisher.publish(vel_msg)
		# time.sleep(1)
		# vel_msg = vel_msg_init()
		# velocity_publisher.publish(vel_msg)
		# vel_msg = vel_msg_init()
		# vel_msg.linear.x = linear_vel
		# velocity_publisher.publish(vel_msg)

		############################################################################################
		
		############################################################################################
		while not rospy.is_shutdown():
			vel_msg = vel_msg_init()
			vel_msg.linear.x= linear_vel
			velocity_publisher.publish(vel_msg)
			time.sleep(1)
			break
		############################################################################################

		############################################################################################
		# print("!!!!!!  Resuming Default Script !!!!!!")
		global active_x, pos_x, curr_x
		global active_y, pos_y, curr_y
		if turn=="left" or turn=="right":
			active_y = not(active_y)
			active_x = not(active_x)
			pos_x = curr_x
			pos_y = curr_y
			if active_y == True:
				print("Y active at", pos_y)
			if active_x == True:
				print("X active at", pos_x)
		turn = "na"		## reinitilize turn value after successfully completing a turn
		############################################################################################
	
	laser_running = False	## mark laser thread completion so that the laser_callback can process new requests

	

## this subscriber function is called every ~millisecond with the LIDAR laser scan as received by the bot
def laser_callback(msg):
	# print("Received laser scan!")
	global laser_running, deviation_running
	# print(msg)
	# print("angle min:", msg.angle_min*180/np.pi, ", angle_max:", msg.angle_max*180/np.pi, ", range_min:", msg.range_min, "range_max:", msg.range_max)
	# print("number of elements in 1 scan:", len(msg.ranges))
	# print(msg.ranges[:-45]+msg.ranges[45:])	## concat 45 to -45 degrees where 0 degree is straight ahead (Note: -ve means to the right)
	# print("::::::::range value in front of bot::::::::::::::::::::::::::::::::::::::::::::::::::::::", msg.ranges[0])
	# dist = msg.ranges[0]	## distance of nearest obj in front of bot
	if laser_running == True:
		## Dont process new laser data till the previous process has been completed
		# print("Previous laser thread not completed yet")
		pass
	else:
		## This code only works for wall right ahead of the bot (since we are only seeing d[0])
		## you should get appropriate distance and threshold according to wall placement for turn (Level 3 task with YOLOnet)
		dist = msg.ranges[0]
		thr = front_wall_thr
		if dist > thr:
			if deviation_running == False:		## do not control motion if deviation running is ON
				# print("==========Go straight==========")
				vel_msg = vel_msg_init()
				vel_msg.linear.x=linear_vel
				velocity_publisher.publish(vel_msg)
		else:
			# print("Starting an obstacle handling thread")
			laser_running = True	## Mark the start of a laser process
			thread.start_new_thread(threaded_obstacle_handling,())
			# threaded_obstacle_handling(d)
			## all time taking process need to be done outside the subscriber function to keep the subscriber and publisher synchronous
			## otherwise the subscriber will resume with old scan values after processing
		


def controller():
	rospy.init_node('hackathonblach', anonymous=True)

	#image_publisher = rospy.Publisher(image_topic, Image)
	## subscribe to image_callback function every time an image is received by the bot
	image_subscriber = rospy.Subscriber(image_topic, Image, image_callback)

	# laser_publisher = rospy.Publisher(laser_topic, LaserScan)
	## subscribe to laser_callback function every time a LIDAR laser scan is received by the bot
	laser_subscriber = rospy.Subscriber(laser_topic, LaserScan, laser_callback)

	odom_sub = rospy.Subscriber(odometry_topic, Odometry, odom_callback)
	rospy.spin()


def main():
	os.system('gnome-terminal -x ' 'roslaunch hackathon fin.launch')	##open the map with the bot and the world
	# time.sleep(5)
	try:
		controller()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	try:
		# Define globals
		image_topic = "/camera/rgb/image_raw"
		laser_topic = "/scan"
		velocity_topic = "/cmd_vel"
		odometry_topic = "/odom"
		velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=1000)
		h="left"
		turn = "na"						## turn initially assigned to "na" meaning not assigned
		laser_running = False
		recognition_running = False
		deviation_running = False
		PI = math.pi					# PI = 3.1415926535897
		linear_vel = 0.4				## linear velocity of bot
		front_wall_thr = 0.55			## threshold for distance from wall at which bot should turn
		orientation = PI/2
		active_x = True
		active_y = False
		pos_x = 9.488605
		curr_x = 9.488605
		pos_y = 11.702337
		curr_y = 11.702337
		dev_thr = 0.1					## deviation threshold for moving bot back to middle of lane
		turn_ang_speed = 30*(PI/180)	## 45 degrees per second
		degree_precision = PI/180		## 1 degree precision while making turns
		main()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python

import numpy as np
import sys
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from autonomy.msg import motors, lines, distance, servos #, leds


class autonomy(object):

	def __init__(self):
		##USER PARAMETERS
		self.dummyParam = 30
		#Lab 1 parameters
		self.uprev = 0
		#Lab 2 parameters
		self.start_time = 0
		self.elapsed_time = 0
		self.drawBox05 = True
		self.drawBox1 = False
		self.safeDistance = 0.4 # Shared with lab 3
		#Lab 3 parameters
		self.obstacle = False
		self.leftDistance = 0
		self.rightDistance = 0
		self.frontDistance = 1
		self.farAwayDist = 0.7
		##Initiate variables
		self.leftLine = 0
		self.midLine = 0
		self.rightLine = 0
		self.distance = 0
		self.leftSpeed = 0
		self.rightSpeed = 0
		self.pan = 0
		self.tilt = 0
		self.bridge = CvBridge()

		#Setup Publishers
		self.motorPub = rospy.Publisher('motors', motors, queue_size=10)
		self.servoPub = rospy.Publisher('servos', servos, queue_size=10)
		#self.LEDpub = rospy.Publisher('leds', leds, queue_size=10)
		self.blobpub = rospy.Publisher('imageProc',Image, queue_size=10)
		#Create Subscriber callbacks
		def lineCallback(data):
			self.leftLine = data.leftLine
			self.midLine = data.midLine
			self.rightLine = data.rightLine

		def distanceCallback(data):
			self.distance = data.distance


		def imageProcessing(data):
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

			##Place image processing code here!
			# Lab 4
			# Setup SimpleBlobDetector parameters.
			params = cv2.SimpleBlobDetector_Params()
			# Change thresholds
			params.minThreshold = 10;
			params.maxThreshold = 200;
			# Filter by Area.
			params.filterByArea = True
			params.minArea = 1500
			# Filter by Circularity
			params.filterByCircularity = False
			params.minCircularity = 0.2
			params.maxCircularity = 0.5
			# Filter by Convexity
			params.filterByConvexity = True
			params.minConvexity = 0.5
			params.maxConvexity = 0.8
			# Filter by Inertia
			params.filterByInertia = False
			params.minInertiaRatio = 0.01
			# Set up a detector
			detector = cv2.SimpleBlobDetector_create(params)
			# Detect blobs
			keypoints = detector.detect(frame)
			self.blobpub.publish(self.bridge.cv2_to_imgmsg(cv2.drawKeypoints(frame, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS),"bgr8"))
		
		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image_rect_color',Image,imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)

		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(10)

	def publishMotors(self):
		motorMsg = motors()
		motorMsg.leftSpeed = self.leftSpeed
		motorMsg.rightSpeed = self.rightSpeed
		rospy.loginfo(motorMsg)
		self.motorPub.publish(motorMsg)

	def publishServo(self):
		servoMsg = servos()
		servoMsg.pan = self.pan
		servoMsg.tilt = self.tilt
		rospy.loginfo(servoMsg)
		self.servoPub.publish(servoMsg)

#	def publishLED(self):
#		LEDmsg = leds()
#		LEDmsg.r1 = 255
#		LEDmsg.g1 = 0
#		LEDmsg.b1 = 0
#		LEDmsg.r2 = 0
#		LEDmsg.g2 = 255
#		LEDmsg.b2 = 0
#		LEDmsg.r3 = 0
#		LEDmsg.g3 = 0
#		LEDmsg.b3 = 255
#		rospy.loginfo(LEDmsg)
#		self.LEDpub.publish(LEDmsg)

	def runner(self):
		while not rospy.is_shutdown():
			## Place code here

			##Leave these lines at the end
			self.publishMotors()
#			self.publishServo()
#			self.publishLED()
			self.rate.sleep()

	def Lab3(self):
    		
		def challenge(): # Define a function to implement continous avoidance
			positions = [1,0,-1,0]	# Array with positions for the servo
			distances = [0,0,0]	# Array to store differences
			speed = 0.5
			turnTime = 2
			if(not self.obstacle): # If there's no obstace keep going straight 
				self.leftSpeed = speed
				self.rightSpeed = speed
				self.publishMotors()
			for i in range(0,len(positions)): # Iterate through the positions to make measurements
				self.pan=positions[i]
				self.publishServo()
				timer = True
				self.start_time = time.time()
				while timer:
					if(i == 3):
	    					distances[i-2] = self.distance
					else:
						distances[i] = self.distance
					self.elapsed_time = time.time()-self.start_time
					if(self.elapsed_time>turnTime):
							timer = False
			# If the distance to the sides is greater than ahead stop
			if(distances[1]<distances[0] or distances[1]<distances[2]):
				self.obstacle = True
			else:
				self.obstacle = False
			# Check which side has a greater distance to object
			if(distances[2]>distances[1] and distances[2]>distances[0]):
				self.leftSpeed = -speed
				self.rightSpeed = speed
			elif(distances[0]>distances[1] and distances[0]>distances[2]):
				self.leftSpeed = speed
				self.rightSpeed = -speed
				
		def simpleAlg():
			speed = 0.2
			lookTime = 2
			turnTime = 0.5
			if(not self.obstacle):	# If there's no obstacle ahead move forward
				self.pan = 0
				self.leftSpeed = speed
				self.rightSpeed = speed
				if(self.distance<self.safeDistance):
					self.obstacle = True
			else:	# Stop
				self.leftSpeed = 0
				self.rightSpeed = 0
				self.publishMotors()
				timer = True
				self.start_time = time.time()
				while timer:
					self.elapsed_time = time.time()-self.start_time
					# Look left and measure distance
					self.pan = 1
					self.publishServo()
					self.leftDistance = self.distance
					if(self.elapsed_time>lookTime):
						timer = False
				timer = True
				self.start_time = time.time()
				while timer:
					self.elapsed_time = time.time()-self.start_time
					# Look right and measure distance
					self.pan = -1
					self.publishServo()
					self.rightDistance = self.distance
					if(self.elapsed_time>lookTime):
						timer = False
				if(self.rightDistance>self.leftDistance):
					timer = True
					self.start_time = time.time()
					while timer:
						self.elapsed_time = time.time()-self.start_time
						# Turn right
						self.leftSpeed = 0.5
						self.rightSpeed = -0.5
						self.publishMotors()
						if(self.elapsed_time>turnTime):
							timer = False
				else:
					timer = True
					self.start_time = time.time()
					while timer:
						self.elapsed_time = time.time()-self.start_time
						# Turn left
						self.leftSpeed = -0.5
						self.rightSpeed = 0.5
						self.publishMotors()
						if(self.elapsed_time>turnTime):
							timer = False
				if(self.distance>self.safeDistance):
					self.obstacle = False

		simpleAlg()

	def Lab2(self):
		# Define a function to do timed control
		# it takes a parameters a time, direction, and speed of motion
		def TimeControl(t,direction,speed):
			timer = True	# Timer is working
			self.start_time = time.time()	# Initialize timer
			while timer:	
				self.elapsed_time = time.time()-self.start_time # Calculate the elapsed time
				while(self.distance < self.safeDistance):	# If the robot runs into an obstacle stop the motors
					self.leftSpeed = 0
					self.rightSpeed = 0
					self.publishMotors()
				if(self.elapsed_time>t):	# If the elapsed time is greater than the set time stop the timer
   					timer = False
				# Define various directions and how the motors should behave in accordance
				elif(direction=="Forward"):
					self.leftSpeed = speed
					self.rightSpeed = speed
				elif(direction=="Backward"):
					self.leftSpeed = -speed
					self.rightSpeed = -speed
				elif(direction=="Left Turn"):
					self.leftSpeed = -speed
					self.rightSpeed = speed
				elif(direction=="Right Turn"):
					self.leftSpeed = speed
					self.rightSpeed = -speed
				self.publishMotors()	# Send the determined signals to the motors
		# Draw a 0.5 meter box counterclockwise, once
		# Define parameter for forward and turn time, and speed
		fTime = 1.2
		turnTime = 0.7
		speed = 0.5
		if(self.drawBox05):	
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Left Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Left Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Left Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Left Turn",speed)
			self.drawBox05 = False	# The 0.5m box should be done only once
		elif(not self.drawBox1):
			self.leftSpeed = 0
			self.rightSpeed = 0	
			self.publishMotors()
		# Define new parameter for the 1m box
		fTime =  1.9
		turnTime = 0.7
		speed = 0.5
		# Draw a 1 meter box clockwise, repeating
		if(self.drawBox1):
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Right Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Right Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Right Turn",speed)
			TimeControl(fTime,"Forward",speed)
			TimeControl(turnTime,"Right Turn",speed)

	def Lab1(self):
		# Set up gains and setpoint
		kp=0.7
		ki=0.1
		setpoint = 0.2
		# Calculate error
		e = self.distance-setpoint
		# Calculate control signal
		up = e*kp
		ui = self.uprev + ki*e*0.1
		u = up + ui
		# Limit the output to the motors and integral action (anti windup)
		if (u>0.7):
			u = 0.7
			ui = 0
		elif (u<-0.7):
			u = -0.7
			ui = 0
		self.leftSpeed = u
		self.rightSpeed = u
		self.uprev = ui
if __name__ == '__main__':
	autonomy().runner()

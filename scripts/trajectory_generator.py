#!/usr/bin/python

import roslib
roslib.load_manifest('qav250')
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Point, Pose
import tf
import math
import time

'''Trajectory generation class
Reads a goal point from the /goal_point topic
Creates a intermediate target and publishes to /refpoint
Also publishes goal & ref points to Rviz
Ref point publisher should move into QAV code.
'''

class TrajectoryGen:
	def __init__(self, method=1, rate=10):

		#Determines which trajectory generation method will be used
		self.method = method
		rospy.loginfo("Trajectory Generation Method: " + str(method))
		#Get trajectory parameters
		self.ramp_distance = rospy.get_param('ramp_distance', 0.05)
		self.position_tolerance = rospy.get_param('position_tolerance', 0.005)
		self.update_freq = rate
		#Subscribe to position and goalpoint
		self.goal_sub = rospy.Subscriber('goal_point', Pose, self.goal_callback)
		#Publisher for interpolated point
		self.ref_pub = rospy.Publisher('refpoint', Point)
		#Initialise with no goal
		self.goal = None
		self.position = Pose()
		self.position.position.x = rospy.get_param('init_x', 0.0)
		self.position.position.y = rospy.get_param('init_y', 0.0)
		self.position.position.z = 0.8
		#RViz broadcaster for goal/ref points
		self.goal_br = tf.TransformBroadcaster()
		self.traj_br = tf.TransformBroadcaster()

	def update(self):

		#Update position data

		#If we have a goal, generate a point in the direction of that goal
		#Also publish the goal to Rviz
		if self.goal is not None:
			self.gen_trajectory()
			self.goal_br.sendTransform((self.goal.position.x, self.goal.position.y, self.goal.position.z),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),rospy.get_namespace()+"/target","world")

	def goal_callback(self, data):
		#Update end goal - might want to put some conditioning on this?
		#Duration data passed as orientation.w
		self.goal = data
		self.duration = data.orientation.w
		self.required_steps = (self.update_freq * self.duration)
		self.completed_steps = 0
		
		error, (dx, dy, dz) = self.goal_distance()
		self.x_step = dx / self.required_steps
		self.y_step = dy / self.required_steps
		self.z_step = dz / self.required_steps

	def goal_distance(self):
		#Calculate distance to goal point
		dx = self.goal.position.x - self.position.position.x
		dy = self.goal.position.y - self.position.position.y
		dz = self.goal.position.z - self.position.position.z

		distance = math.sqrt(dx*dx + dy*dy + dz*dz)

		return (distance, (dx, dy, dz))

	def gen_trajectory(self):
		#Use chosen trajectory generation method
		if self.method == 1:
			self.ramp_trajectory()
		else:
			rospy.loginfo("Trajectory method not recognised")

	def ramp_trajectory(self):
		# desired positions increase with time
		# - set point remains a constant distance in front
		# - RAMP_DISTANCE defines required offset
			# set new point to set distance ahead
		if self.completed_steps < self.required_steps:

			self.position.position.x += self.x_step
			self.position.position.y += self.y_step
			self.position.position.z += self.z_step

			self.completed_steps += 1
			
			refpoint = Point()
			refpoint.x = self.position.position.x
			refpoint.y = self.position.position.y
			refpoint.z = self.position.position.z

			self.ref_pub.publish(refpoint)
			rospy.loginfo("Intermediate target is: (%2.2f, %2.2f, %2.2f)" % (refpoint.x, refpoint.y, refpoint.z))

			

def main():
	# "main" code - sloppy but ok for now
	rospy.init_node('trajectoryGenerator', anonymous=True)
	TG = TrajectoryGen()
	update_freq = rospy.get_param('/update_frequency', 10)
	r = rospy.Rate(update_freq)
	TG.rate = update_freq

	while not rospy.is_shutdown():
		TG.update()
		r.sleep()

if __name__ == "__main__":
	main()

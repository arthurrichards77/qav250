#!/usr/bin/env python
import roslib
roslib.load_manifest('qav250')
import rospy
import numpy
import rospid
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import euler_from_quaternion

class qav250:

  def __init__(self):
    self.node = rospy.init_node('qav250', anonymous=True)
    # publisher for RC commands
    self.rc_pub = rospy.Publisher('rcctrl', numpy_msg(Floats))
    # subscriber for Vicon data
    self.vicon_sub = rospy.Subscriber('drone', TransformStamped, self.vicon_callback)
    # PID controller for each axis
    self.pitch_pid = rospid.rospid(0.075,0.0,0.11,'pitch') # pitch
    self.roll_pid = rospid.rospid(0.075,0.0,0.11,'roll') # roll
    self.thrust_pid = rospid.rospid(0.2,0.03,0.15,'thrust') # height
    self.yaw_pid = rospid.rospid(0.25,0.0,0.0,'yaw') # yaw
    # freeze the thrust integrator until flying    
    self.thrust_pid.freeze_integrator()

  def vicon_callback(self,data):
    #rospy.loginfo('%s',data)
    # extract the time in seconds
    t = data.header.stamp.to_sec()
    # t = data.header.stamp
    # only enable integral action when over 20cm off ground - to avoid wind-up
    if data.transform.translation.z>0.2:
      self.thrust_pid.enable_integrator()
    else:
      self.thrust_pid.freeze_integrator()
      # Also clear integrator, to restart experiment
      self.thrust_pid.resetintegrator(0.0)
      
    # update each control loop
    #cmd_orientation = Quaternion()
    #cmd_orientation.x = 0
    #cmd_orientation.y = 0
    #cmd_orientation.z = -0.87
    #cmd_orientation.w = 0
    #cmd_orientation = cmd_orientation - data.transform.rotation
    quaternion = (data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    u_roll = self.roll_pid.update((-data.transform.translation.x), 0.0, t)
    u_pitch = self.pitch_pid.update(-data.transform.translation.y, 0.0, t)
    #u_yaw = self.yaw_pid.update((-data.transform.rotation.z)-0.78, 0.0, t)
    #u_yaw = self.yaw_pid.update(cmd_orientation.z, 0.0, t)
    u_yaw = self.yaw_pid.update(-euler[2]-1.57, 0.0, t)
    u_thrust = self.thrust_pid.update(data.transform.translation.z, 0.8, t)
    # centre around 0.5 and limit
    c_roll = 0.5 + rospid.saturate(u_roll,0.25)
    c_pitch = 0.5 + rospid.saturate(u_pitch,0.25)
    c_yaw = 0.5 + rospid.saturate(u_yaw,0.25)
    # except thrust, centered around something bigger
    c_thrust = 0.6 + rospid.saturate(u_thrust,0.15)
    # print data.transform.translation.x, 0.0, t, u_roll
    rospy.loginfo('(Roll pitch yaw) = (%f %f %f) Thrust = %f',u_roll, u_pitch, u_yaw, u_thrust)
    # rospy.loginfo('Thrust integrator = %f', self.thrust_pid.read_integrator())
    # compile into message for RC bridge
    # channels: 1 = Roll (pos right) 2 = Pitch (pos forward) 3 = Thrust (pos up) 4 = Yaw (pos right / CW)
    rc_ctrl = numpy.array([c_roll,c_pitch,c_thrust,c_yaw,0.0,0.0,0.0,0.0], dtype=numpy.float32)
    # send it to the bridge
    self.rc_pub.publish(rc_ctrl)

if __name__ == "__main__":
  q = qav250()
  rospy.spin()  
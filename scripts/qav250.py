#!/usr/bin/env python
import roslib
roslib.load_manifest('qav250')
import rospy
import numpy
import rospidlib
import tf
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import TransformStamped, Point

class qav250:

  def __init__(self):
    self.node = rospy.init_node('qav250', anonymous=True)
    # publisher for RC commands
    self.rc_pub = rospy.Publisher('rcctrl', numpy_msg(Floats))
    # subscriber for Vicon data
    self.vicon_sub = rospy.Subscriber('drone', TransformStamped, self.vicon_callback)
    # subscriber for reference point input
    self.point_sub = rospy.Subscriber('refpoint', Point, self.ref_callback)
    # PID controller for each axis
    self.pitch_pid = rospidlib.Rospid(0.075,0.0,0.11,'~pitch') # pitch
    self.roll_pid = rospidlib.Rospid(0.075,0.0,0.11,'~roll') # roll
    self.thrust_pid = rospidlib.Rospid(0.2,0.03,0.15,'~thrust') # height
    self.yaw_pid = rospidlib.Rospid(0.25,0.0,0.0,'~yaw') # yaw
    # freeze the thrust integrator until flying    
    self.thrust_pid.freeze_integrator()
    # point reference input
    self.ref_point = Point()
    # default is at origin, 0.8m off ground
    self.ref_point.z = 0.8
    # get min/max points
    self.min_x = rospy.get_param('/min_x', -5.0)
    self.min_y = rospy.get_param('/min_y', -5.0)
    self.min_z = rospy.get_param('/min_z', 0.0)
    self.max_x = rospy.get_param('/max_x', 5.0)
    self.max_y = rospy.get_param('/max_y', 5.0)
    self.max_z = rospy.get_param('/max_z', 5.0)
    self.rel_threshold = rospy.get_param('/rel_threshold', 2.5)
    #broadcast target point to RViz
    self.target_br = tf.TransformBroadcaster()

  def ref_callback(self,data):
    #Sanity check new refpoint
    if self.check_absolute_ref(data):
        if self.check_relative_ref(data):
            self.ref_point = data
            rospy.loginfo('New reference received: (%f, %f, %f)', self.ref_point.x, self.ref_point.y, self.ref_point.z)
        else:
            rospy.loginfo('New reference outside of relative limits')
    else:
        rospy.loginfo('New reference outside of absolute limits')

  def check_absolute_ref(self,data):
    return data.x >= self.min_x and data.x <= self.max_x and data.y >= self.min_y and data.y <= self.max_y and data.z >= self.min_z and data.z <= self.max_z

  def check_relative_ref(self, data):
    if hasattr(self, 'current_pos'):
      xdiff = self.current_pos.x - data.x
      ydiff = self.current_pos.y - data.y
      zdiff = self.current_pos.z - data.z

      rospy.loginfo('relative reference %f', np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff))
      return np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff) < self.rel_threshold
    else:
      return True

  def vicon_callback(self,data):
    self.current_pos = Point(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    #publish current target point to Rviz
    self.target_br.sendTransform((self.ref_point.x, self.ref_point.y, self.ref_point.z), 
		(0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "control_ref_point", "world")
    #rospy.loginfo('%s',data)
    # extract the time in seconds
    t = data.header.stamp.to_sec()
    # t = data.header.stamp
    # only enable integral action when over 20cm off ground - to avoid wind-up
    if data.transform.translation.z>0.3:
      self.pitch_pid.enable_integrator()
      self.roll_pid.enable_integrator()
      self.yaw_pid.enable_integrator()
      self.thrust_pid.enable_integrator()
    else:
      self.pitch_pid.freeze_integrator()
      self.roll_pid.freeze_integrator()
      self.yaw_pid.freeze_integrator()
      self.thrust_pid.freeze_integrator()
    # update each control loop
    u_roll = self.roll_pid.update((-data.transform.translation.y), (-self.ref_point.y), t)
    u_pitch = self.pitch_pid.update(data.transform.translation.x, self.ref_point.x, t)
    u_yaw = self.yaw_pid.update((-data.transform.rotation.z), 0.0, t)
    u_thrust = self.thrust_pid.update(data.transform.translation.z, self.ref_point.z, t)
    # centre around 0.5 and limit
    c_roll = 0.5 + rospidlib.saturate(u_roll,0.25)
    c_pitch = 0.5 + rospidlib.saturate(u_pitch,0.25)
    c_yaw = 0.5 + rospidlib.saturate(u_yaw,0.25)
    # except thrust, centered around something bigger
    c_thrust = 0.6 + rospidlib.saturate(u_thrust,0.15)
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



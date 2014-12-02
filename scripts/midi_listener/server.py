#!/usr/bin/env python
from OSC import OSCServer
import sys
from time import sleep
import rospy
import roslib
from std_msgs.msg import Empty

rospy.init_node('midi_listener')
midi_pub = rospy.Publisher('drone/drop', Empty)

server = OSCServer( ("localhost", 7110) )
server.timeout = 0
run = True

# this method of reporting timeouts only works by convention
# that before calling handle_request() field .timed_out is 
# set to False
def handle_timeout(self):
    self.timed_out = True

# funny python's way to add a method to an instance of a class
import types
server.handle_timeout = types.MethodType(handle_timeout, server)

def user_callback(path, tags, args, source):
    # which user will be determined by path:
    # we just throw away all slashes and join together what's left
    user = ''.join(path.split("/"))
    # tags will contain 'fff'
    # args is a OSCMessage with data
    # source is where the message came from (in case you need to reply)
    print ("Message received")
    midi_pub.publish() 

server.addMsgHandler( "/startup", user_callback )

# Loop forever
while run:
    # do the game stuff:
    sleep(1)

server.close()

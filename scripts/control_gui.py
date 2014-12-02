#!/usr/bin/python
import time
import rospy
from Tkinter import *
import roslib
roslib.load_manifest('qav250')
import tf
from geometry_msgs.msg import Point, TransformStamped, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty
import midi_listener

class App:
	def __init__(self, master):
		frame=Frame(master)
		frame.pack()

		#Create publisher for goal point
		self.goal_pub = rospy.Publisher('goal_point', Pose)

		#Create publisher to freeze integator
                self.freeze_int_pub = rospy.Publisher('freeze_int', Bool)

		#Create subscriber to listen for drop message
		self.drop_sub = rospy.Subscriber('drop', Empty, self.drop_callback)

		#Initial position
		self.position = Pose()
		self.position.position.x = rospy.get_param('init_x', 0.0)
		self.position.position.y = rospy.get_param('init_y', 0.0)
		self.position.position.z = rospy.get_param('init_z', 0.8)

		#Get Min/Max values for x,y,z
		self.min_x = rospy.get_param('min_x', -5.0)
		self.max_x = rospy.get_param('max_x', 5.0)
		self.min_y = rospy.get_param('min_y', -5.0)
		self.max_y = rospy.get_param('max_y', 5.0)
		self.min_z = rospy.get_param('min_z', 0.0)
		self.max_z = rospy.get_param('max_z', 5.0)

		#Get step sizes
		self.x_step = rospy.get_param('x_step', 0.1)
		self.y_step = rospy.get_param('y_step', 0.1)
		self.z_step = rospy.get_param('z_step', 0.1)

		#Get up drell time for drop manoeuvre
		self.up_dwell = rospy.get_param('up_dwell', 0.2)

		#Make quit button
		self.quit_button = Button(frame, text="Quit", command=frame.quit)
		self.quit_button.grid(row=1000, column=1000)

		#Make stop button
		self.stop_button = Button(frame, text="Stop", command=self.stop_callback)
		self.stop_button.grid(row=101, column=1)

		#Directional push buttons
		self.push_buttons = list()
		#+z
		self.up_button = Button(frame, text="+z", command=lambda m="+z": self.push_callback(m))
		self.up_button.grid(row=0, column=3)
		self.push_buttons.append(self.up_button)
		#-z
		self.down_button = Button(frame, text="-z", command=lambda m="-z": self.push_callback(m))
		self.down_button.grid(row=2, column=3)
		self.push_buttons.append(self.down_button)
		#+y
		self.left_button = Button(frame, text="+y", command=lambda m="+y": self.push_callback(m))
		self.left_button.grid(row=0, column=1)
		self.push_buttons.append(self.left_button)
		#-y
		self.right_button = Button(frame, text="-y", command=lambda m="-y": self.push_callback(m))
		self.right_button.grid(row=2, column=1)
		self.push_buttons.append(self.right_button)
		#+x
		self.forward_button = Button(frame, text="+x", command=lambda m="+x": self.push_callback(m))
		self.forward_button.grid(row=1, column=2)
		self.push_buttons.append(self.forward_button)
		#-x
		self.back_button = Button(frame, text="-x", command=lambda m="-x": self.push_callback(m))
		self.back_button.grid(row=1, column=0)
		self.push_buttons.append(self.back_button)

		#Buttons for landing / take-off
		self.land_button = Button(frame, text="Take_off", command=self.takeoff_callback)
		self.land_button.grid(row=100, column=0)

		self.takeoff_button = Button(frame, text="Land", command=self.land_callback)
		self.takeoff_button.grid(row=101, column=0)

		#Drop on to target
		self.drop_button = Button(frame, text="Play Sound", command=lambda m="-drop": self.push_callback(m))
		self.drop_button.grid(row=100, column=1)

		#Fixed positional input entry boxes
		#Create label
		self.x_label = Label(frame, text='X position')
		self.x_label.grid(row=0, column=100, padx=10, pady=10)
		#Create entry box
		vcmd = (frame.register(self.validate_goal), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
		self.x_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.x_entry.grid(row=2, column=100, padx=10, pady=10)

		#Create label
		self.y_label = Label(frame, text='Y position')
		self.y_label.grid(row=0, column=101, padx=10, pady=10)
		#Create entry box
		self.y_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.y_entry.grid(row=2, column=101, padx=10, pady=10)

		#Create label
		self.z_label = Label(frame, text='Z position')
		self.z_label.grid(row=0, column=102, padx=10, pady=10)
		#Create entry box
		self.z_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.z_entry.grid(row=2, column=102, padx=10, pady=10)

		#Send command button
		self.send_pos_button = Button(frame, text="Send", command=self.send_position)
		self.send_pos_button.grid(row=4, column=100)

		#Dispay status of command
		self.send_status_text = StringVar()
		self.send_status_text.set('')
		self.send_status_label = Label(frame, textvariable=self.send_status_text)
		self.send_status_label.grid(row=4, column=101)

		#Position feedback
		self.x_pos_text = StringVar()
		self.x_pos_text.set('0.0')
		self.x_pos_label = Label(frame, textvariable=self.x_pos_text)
		self.x_pos_label.grid(row=1, column=100)

		self.y_pos_text = StringVar()
		self.y_pos_text.set('0.0')
		self.y_pos_label = Label(frame, textvariable=self.y_pos_text)
		self.y_pos_label.grid(row=1, column=101)

		self.z_pos_text = StringVar()
		self.z_pos_text.set('0.0')
		self.z_pos_label = Label(frame, textvariable=self.z_pos_text)
		self.z_pos_label.grid(row=1, column=102)


		#Drop dwell entry

		self.up_dwell_label = Label(frame, text='Up dwell time')
		self.up_dwell_label.grid(row=101, column=100, padx=10, pady=10) 
		self.up_dwell_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.up_dwell_entry.grid(row=101, column=101, padx=10, pady=10)

		self.update_up_dwell = Button(frame, text="Update up dwell", command=self.update_up_dwell)
		self.update_up_dwell.grid(row=101, column=102)

	def update_up_dwell(self):
		try:
			self.up_dwell = float(self.up_dwell_entry.get())
		except ValueError:
			rospy.loginfo("Invalid up dwell sent")

	def stop_callback(self):
		self.goal_pub.publish(self.position)

	def add_goal(self):
		try:
			x = float(self.x_entry.get())
			y = float(self.y_entry.get())
			z = float(self.z_entry.get())

			label = Label()
			del_button = Button()
		except ValueError:
			self.send_status_text.set('Co-ordinate is not float')
			return

	def push_callback(self, button=''):
		if not hasattr(self, 'position'):
			rospy.loginfo("No position - push not sent")
			return
		else:
			self.position.orientation.w = 0.1
			if button == '+z':
				self.position.position.z += self.z_step
			elif button == '-z':
				self.position.position.z -= self.z_step
			elif button == '+x':
				self.position.position.x += self.x_step
			elif button == '-x':
				self.position.position.x -= self.x_step
			elif button == '+y':
				self.position.position.y += self.y_step
			elif button == '-y':
				self.position.position.y -= self.y_step
			elif button == '-drop':
                                self.freeze_int_pub.publish(True)
				old_pos_z = self.position.position.z
				self.position.position.z = 3.0
				self.position.orientation.w = 0.1
				self.send_goal()
				time.sleep(self.up_dwell)
				self.position.position.z = old_pos_z
				self.position.orientation.w = 0.1
                                self.freeze_int_pub.publish(False)


		self.send_goal()
		rospy.loginfo("Sent push" + button)

	def send_position(self):
		
		#Ensure each co-ordinate is a valid float
		try:
			x = float(self.x_entry.get())
			y = float(self.y_entry.get())
			z = float(self.z_entry.get())

			#If we have floats, ensure co-ordinates are within valid range
			if x < self.min_x:
				self.send_status_text.set('X co-ordinate too low')
				return
			if x > self.max_x:
				self.send_status_text.set('X co-ordinate too big')
				return
			if y < self.min_y:
				self.send_status_text.set('Y co-ordinate too low')
				return
			if y > self.max_y:
				self.send_status_text.set('Y co-ordinate too big')
				return
			if z < self.min_z:
				self.send_status_text.set('Z co-ordinate too low')
				return
			if z > self.max_z:
				self.send_status_text.set('Z co-ordinate too big')
				return

			#We have floats within the valid area - send the position
			self.position.position.x = x
			self.position.position.y = y
			self.position.position.z = z
			self.position.orientation.w = 1

			self.send_goal()
			self.send_status_text.set('Position sent')
			rospy.loginfo("New position sent:(%2.2f, %2.2f, %2.2f" % (new_position.position.x, new_position.position.y, new_position.position.z))

		except ValueError:
			self.send_status_text.set('Co-ordinate is not float')
			return

	def land_callback(self):
		print "Landing"

	def takeoff_callback(self):
		print "Taking off"

	def validate_goal(self, d, i, P, s, S, v, V, W):
		#Allow numbers
		if (S in '0123456789'):
			return True
		#Allow a minus sign as the first character
		elif S == '-' and i == '0':
			return True
		#Allow a single dot character
		elif S == '.' and P.count('.') <= 1:
			return True
		#Allow deletion
		elif P == '':
			return True
		#Reject everything else
		else:
			return False

	def send_goal(self):

		self.goal_pub.publish(self.position)
		self.x_pos_text.set("%2.2f" % self.position.position.x)
		self.y_pos_text.set("%2.2f" % self.position.position.y)
		self.z_pos_text.set("%2.2f" % self.position.position.z)

	def drop_callback(self, data):
		m= '-drop'
		self.push_callback(m)
	
		
def main():
	# "main" code - sloppy but ok for now
	rospy.init_node('control_gui', anonymous=True)
	root = Tk()
	
	# show the namespace in the window title
	root.wm_title(rospy.get_namespace())
	app = App(root)
	root.mainloop()
	root.destroy()

if __name__ == "__main__":
	main()


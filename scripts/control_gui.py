#!/usr/bin/python
import time
import rospy
from Tkinter import *
import roslib
roslib.load_manifest('qav250')
import tf
from geometry_msgs.msg import Point, TransformStamped, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class App:
	def __init__(self, master):
		frame=Frame(master)
		frame.pack()

		#Create publisher for push point
		self.push_pub = rospy.Publisher('goal_point', Pose)
		#Create publisher for goal point
		self.goal_pub = rospy.Publisher('goal_point', Pose)
		#Create publisher to freeze integator
                self.freeze_int_pub = rospy.Publisher('freeze_int', Bool)
		#Initial position
		self.position = Pose()
		self.position.position.x = rospy.get_param('init_x', 0.0)
		self.position.position.y = rospy.get_param('init_y', 0.0)
		self.position.position.z = 0.8

		#Get Min/Max values for x,y,z - Question mark over it is better to use these, or have some kind of check that a message has been set.
		self.min_x = rospy.get_param('/min_x', -5.0)
		self.max_x = rospy.get_param('/max_x', 5.0)
		self.min_y = rospy.get_param('/min_y', -5.0)
		self.max_y = rospy.get_param('/max_y', 5.0)
		self.min_z = rospy.get_param('/min_z', 0.0)
		self.max_z = rospy.get_param('/max_z', 2.0)

		#Get step sizes
		self.x_step = rospy.get_param('/x_step', 0.1)
		self.y_step = rospy.get_param('/y_step', 0.1)
		self.z_step = rospy.get_param('/z_step', 0.1)

		#Get dwell time for drop manouevre
		self.drop_dwell = rospy.get_param('/drop_dwell', 0.3)

		#Make quit button
		self.quit_button = Button(frame, text="Quit", command=frame.quit)
		self.quit_button.grid(row=0, column=1)

		#Make stop button
		self.stop_button = Button(frame, text="Stop", command=self.stop_callback)
		self.stop_button.grid(row=0, column=0)

		#Directional push buttons
		self.push_buttons = list()

		self.up_button = Button(frame, text="+z", command=lambda m="+z": self.push_callback(m))
		self.up_button.grid(row=1, column=0)
		self.push_buttons.append(self.up_button)

		self.down_button = Button(frame, text="-z", command=lambda m="-z": self.push_callback(m))
		self.down_button.grid(row=1, column=1)
		self.push_buttons.append(self.down_button)

		self.left_button = Button(frame, text="+y", command=lambda m="+y": self.push_callback(m))
		self.left_button.grid(row=2, column=0)
		self.push_buttons.append(self.left_button)

		self.right_button = Button(frame, text="-y", command=lambda m="-y": self.push_callback(m))
		self.right_button.grid(row=2, column=1)
		self.push_buttons.append(self.right_button)

		self.forward_button = Button(frame, text="+x", command=lambda m="+x": self.push_callback(m))
		self.forward_button.grid(row=3, column=0)
		self.push_buttons.append(self.forward_button)

		self.back_button = Button(frame, text="-x", command=lambda m="-x": self.push_callback(m))
		self.back_button.grid(row=3, column=1)
		self.push_buttons.append(self.back_button)

		#Buttons for landing / take-off
		self.land_button = Button(frame, text="Land", command=self.land_callback)
		self.land_button.grid(row=4, column=0)

		self.takeoff_button = Button(frame, text="Take-off", command=self.takeoff_callback)
		self.takeoff_button.grid(row=4, column=1)

		#Fixed positional input entry boxes
		#Create label
		self.x_label = Label(frame, text='X position')
		self.x_label.grid(row=1, column=2, padx=10, pady=10)
		#Bind validation to entry box when object is created
		vcmd = (frame.register(self.validate_goal), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
		self.x_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.x_entry.grid(row=1, column=3, padx=10, pady=10)

		self.y_label = Label(frame, text='Y position')
		self.y_label.grid(row=2, column=2, padx=10, pady=10) 
		self.y_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.y_entry.grid(row=2, column=3, padx=10, pady=10)

		self.z_label = Label(frame, text='Z position')
		self.z_label.grid(row=3, column=2, padx=10, pady=10) 
		self.z_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.z_entry.grid(row=3, column=3, padx=10, pady=10)

		self.send_pos_button = Button(frame, text="Send", command=self.send_position)
		self.send_pos_button.grid(row=4, column=2)

		self.send_status_text = StringVar()
		self.send_status_text.set('')
		self.send_status_label = Label(frame, textvariable=self.send_status_text)
		self.send_status_label.grid(row=4, column=3)

		self.add_pos_button = Button(frame, text="Add", command=self.add_goal)
		self.add_pos_button.grid(row=5, column=2)

		self.drop_button = Button(frame, text="Drop", command=lambda m="-drop": self.push_callback(m))
		self.drop_button.grid(row=5, column=3)

		#Position feedback
		self.x_pos_text = StringVar()
		self.x_pos_text.set('0.0')
		self.x_pos_label = Label(frame, textvariable=self.x_pos_text)
		self.x_pos_label.grid(row=6, column=5)
		self.x_pos_label = Label(frame, text="X")
		self.x_pos_label.grid(row=5, column=5)

		self.y_pos_text = StringVar()
		self.y_pos_text.set('0.0')
		self.y_pos_label = Label(frame, textvariable=self.y_pos_text)
		self.y_pos_label.grid(row=6, column=6)
		self.y_pos_label = Label(frame, text="Y")
		self.y_pos_label.grid(row=5, column=6)

		self.z_pos_text = StringVar()
		self.z_pos_text.set('0.0')
		self.z_pos_label = Label(frame, textvariable=self.z_pos_text)
		self.z_pos_label.grid(row=6, column=7)
		self.z_pos_label = Label(frame, text="Z")
		self.z_pos_label.grid(row=5, column=7)

		#Drop dwell entry
		self.dwell_label = Label(frame, text='Drop dwell time')
		self.dwell_label.grid(row=10, column=6, padx=10, pady=10) 
		self.dwell_entry = Entry(frame, validate="key", validatecommand=vcmd)
		self.dwell_entry.grid(row=10, column=7, padx=10, pady=10)

		self.update_dwell = Button(frame, text="Update dwell", command=self.update_dwell)
		self.update_dwell.grid(row=10, column=5)

	def update_dwell(self):
		try:
			self.drop_dwell = float(self.dwell_entry.get())
		except ValueError:
			rospy.loginfo("Invalid drop sent")

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
				self.position.position.z = 0.1
				self.position.orientation.w = 0.1
				self.send_goal()
				time.sleep(self.drop_dwell)
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
		#Reject everything else
		else:
			return False

	def send_goal(self):
		self.goal_pub.publish(self.position)
		self.x_pos_text.set("%2.2f" % self.position.position.x)
		self.y_pos_text.set("%2.2f" % self.position.position.y)
		self.z_pos_text.set("%2.2f" % self.position.position.z)


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


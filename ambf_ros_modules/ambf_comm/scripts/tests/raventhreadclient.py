from ambf_client import Client
from pynput.keyboard import Key, Listener
import time
import math
import numpy as np
import threading


print("\n\n Booting Up.. \n")

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()
homed = [False, False]

print("\n\n Connecting Raven2 Arms...\n")
#connect to raven arms
l_handle =  _client.get_obj_handle('raven_2/base_link_L')
r_handle = _client.get_obj_handle('raven_2/base_link_R')

#variable declarations for Raven2 Movement

start_jp = np.zeros((2,7)) #indexed at 0
delta_jp = np.zeros((2,7)) 

home_joints = [math.pi/3, math.pi*3/5, -0.09, math.pi*3/4, 0, math.pi/6, math.pi/6]
dance_scale_joints = [0.3, 0.3, 0.06, 0.3, 1.2, math.pi/6, math.pi/6]
loop_rate = 1000
raven_joints = 7

user_input = ""

#listener = Listener(on_press = on_press)

time.sleep(0.2)

# =============== Methods ===============






# =============== Threading Classes ===============

class Control:


	def __init__(self):
		self.sine_dance = False
		self.home = False
		self.print_status = False
		self.view_j_pos = False
		self.view_children = False
		self.quit = False
		self.listener = None
	def print_controls(self):
		print("\nsine_dance = " + str(self.sine_dance))
		print("\nhome = " + str(self.home))
		print("\nprint_status = " + str(self.print_status))
		print("\nview_j_pos = " + str(self.view_j_pos))
		print("\nview_children = " + str(self.view_children))
		print("\nquit = " + str(self.quit))


	def start_system(self):
		self.listener = Listener(on_press = self.on_press)
		self.listener.start()
		print("System is ready\n")
	def on_press(self, key):
		user_input = int(key)
		print("Key press detected: " + str(user_input))
		if user_input == 1:
			self.do_sine_dance()
		elif user_input == 2:
			self.do_home()
	def do_sine_dance(self):
		self.reset()
		self.sine_dance = True
		
	def do_home(self):
		self.reset()
		self.home = True

	def do_print_status(self):
		self.reset()
		self.print_status = True

	def do_view_j_pos(self):
		self.reset()
		self.view_j_pos = True
	
	def view_children(self):
		self.reset()
		self.view_children = True
	
	def do_quit(self):
		self.reset()
		self.quit = True

	def reset(self):
		self.sine_dance = False
		self.home = False
		self.print_status = False
		self.view_j_pos = False
		self.view_children = False
		self.quit = False

class home (threading.Thread):
	def __init__(self, threadID, name, control, l_handle, r_handle):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.control = control
		self.l_handle = l_handle
		self.r_handle = r_handle

	def run(self):
		#get lock to sync threads
		threadLock.acquire()
		while True:

			if self.control.home:
				print("\nStarting " + self.name)
				for i in range(loop_rate):
					if not i:
						print("check1")
						homed[0] = self.go_home(1, 1, i)
						homed[1] = self.go_home(1, 0, i)
					else:
						print("check2")
						homed[0] = self.go_home(0, 1, i)
						homed[1] = self.go_home(0, 0, i)
					time.sleep(0.01)
				if homed[0] and homed[1]:
					print("\n Raven2 is HOMED\n")
					control.reset()
		threadLock.release()


	def go_home(self,first_entry, arm, count):
		'''
		first entry --> bool (0 for no, 1 for yes)
		arm --> bool (0 for left, 1 for right)
		count --> int
		'''
		if not arm: #decides which arm...
				state = self.l_handle
		else:
				state = self.r_handle
		#if first time calling go home
		if(first_entry):
			for i in range(state.get_num_joints()):
				start_jp[arm][i] = state.get_joint_pos(i)
				delta_jp[arm][i] = home_joints[i] - state.get_joint_pos(i)
		#gradualizes movement from a to b
		scale = min(1.0*count/loop_rate, 1.0)
		#array containing distance to go to start point
		diff_jp = [0,0,0,0,0,0,0]

		#sets position for each joint
		for i in range(state.get_num_joints()):
			state.set_joint_pos(i, scale * delta_jp[arm][i] + start_jp[arm][i])
			print(state.get_all_joint_pos())
			diff_jp[i] = abs(home_joints[i] - state.get_joint_pos(i))
		#in progress, indicates when arm is honed

		
		max_value = np.max(diff_jp)

		if(max_value < 0.1):
			#for i in range(7):
				#print("joint num " + str(i) + "joint position " + str(state.get_joint_pos(i)))
			homed[arm] = True 

		else: 
			homed[arm] = False 
		return homed[arm]
class sine_dance (threading.Thread):
	def __init__(self, threadID, name, control, l_handle, r_handle):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.control = control
		self.r_handle = r_handle
		self.l_handle = l_handle
	def run(self):
		threadLock.acquire()
		while True:
			if self.control.sine_dance:
				print("\nStarting " + self.name)
				if not homed[0] or not homed[1]:
					print("\nERROR: Raven2 Not Homed")
				else:
					print("\n Beginning Sine Dance...\n")
					rc = [0,0]
					rampup_count = np.array(rc)
					i = 0
					while control.sine_dance:
						if i == 0:
							self.sine_dance(1, 1, i, rampup_count)
							self.sine_dance(1, 0, i, rampup_count)
						else:
							self.sine_dance(0, 1, i, rampup_count)
							self.sine_dance(0, 0, i, rampup_count)
						i += 1
						time.sleep(0.01)
		threadLock.release()
	def sine_dance(self,first_entry, arm, count, rampup_count): #raven_2 MUST BE HOMED
		speed = 1.00/loop_rate
		rampup_speed = 0.5/loop_rate
		if not arm:
			state = self.l_handle
		else:
			state = self.r_handle
		for i in range(raven_joints):
			offset = (i+arm)*math.pi/2
			#print("offset = " + str(offset))
			rampup = min(rampup_speed*rampup_count[arm], 1.0)
			#print("rampup = " + str(rampup))
			state.set_joint_pos(i, rampup*dance_scale_joints[i]*math.sin(speed*(count+offset))+home_joints[i])
			#if(i == 0):
			#	print("actual joint position = " + str(state.get_joint_pos(0)))
			#	print("commanded join position = " + str(dance_scale_joints[0]*math.sin(speed*(count+offset))+home_joints[0]))
			rampup_count[arm] += 1
'''
class userInput (threading.Thread):
	def __init__(self,threadID,name,control,listener):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.control = control
		self.listener = listener
		self.input = None
	def run(self):
		threadLock.acquire()
		
	def on_press(key):
		self.input = str(key)
		print(self.input)
'''

threadLock = threading.Lock()
threads = []

def main():
	#Create control

	control = Control()
	control.start_system()
	#Create Threads
	homing_thread = home(1, "homing", control, l_handle, r_handle)
	sine_dance_thread = sine_dance(2, "sine dance", control, l_handle, r_handle)

	homing_thread.start()
	sine_dance_thread.start()

	control.on_press(2)
	'''
	print("\n\n Homing Raven2 Arms...\n")
	for i in range(loop_rate):
		if not i:
			homed[0] = go_home(1, 1, i)
			homed[1] = go_home(1, 0, i)
		else:
			homed[0] = go_home(0, 1, i)
			homed[1] = go_home(0, 0, i)
		time.sleep(0.01)
	if homed[0] and homed[1]:
		print("\n Raven2 is HOMED\n")
	else:
		print("Error: Raven2 Cannot Be Homed\n")
		exit()
	'''
	raw_input("\n\n -------------- \n Raven Client is ready\n -------------- ")



if __name__ == "__main__":
	main()

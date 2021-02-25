# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import numpy as np

#from pynput import keyboard

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

time.sleep(0.2)


#======= METHODS =========
def go_home(first_entry, arm, count):
	'''
	first entry --> bool (0 for no, 1 for yes)
	arm --> bool (0 for left, 1 for right)
	count --> int
	'''
	if not arm: #decides which arm...
			state = l_handle
	else:
			state = r_handle
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
def sine_dance(first_entry, arm, count, rampup_count): #raven_2 MUST BE HOMED
	speed = 1.00/loop_rate
	rampup_speed = 0.5/loop_rate
	if not arm:
		state = l_handle
	else:
		state = r_handle
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

def display_input_options():
	print("\n--------------------------\n")
	print(" 'Q' --> QUIT\n")
	print(" 'P' --> VIEW J POS\n")
	print(" 'C' --> VIEW CHILDREN\n")
	print(" 'H' --> HOME \n")
	print(" 'S' --> SINE DANCE\n")
def display_pos():
	print('\nLeft Base Pos:')
	print(l_handle.get_pos())
	print('\nLeft Joint Pos:')
	print(l_handle.get_all_joint_pos())
	print('\nRight Base Pos:')
	print(r_handle.get_pos())
	print('\nRight Joint Pos:')
	print(r_handle.get_all_joint_pos())

def display_children():
	l_num_joints = l_handle.get_num_joints() # Get the number of joints of this object
	r_num_joints = l_handle.get_num_joints()
	l_children = l_handle.get_children_names()
	r_children = r_handle.get_children_names()

	print('Left Num Joints: ')
	print(l_num_joints)

	print(' ')
	print('Right Num Joints: ')
	print(r_num_joints)

	print(' ')
	print('Left Children: ')
	print(l_children)

	print(' ')
	print('Right Children: ')
	print(r_children)
'''
def on_press(key):
	if key == keyboard.Key.esc:
		return False
	try:
		k = key.char
	except:
		k = key.name
	if k in ['enter']:
		return False
'''
def main():
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
	raw_input("\n\n -------------- \n Raven Client is ready\n -------------- ")
	user_input = ""
	print("")
	while True:
		display_input_options()
		user_input = input("Enter an option for movement...\n")

		if user_input == 'Q':
			user_input = input("Are you sure you would like to quit? ('y'/'n')\n")
			if user_input == 'y':
				break
		elif user_input == 'H':
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
		elif user_input == 'S':
			if not homed[0] or not homed[1]:
				print("\n ERROR: Raven2 NOT homed. You may not call sine dance unless Raven2 is homed.\n")
			else:
				#listener = keyboard.listener(on_press = on_press)
				#listener.start()
				#listener.join()
				print("\n Beginning Sine Dance...\n")
				rc = [0,0]
				rampup_count = np.array(rc)
				i = 0
				while True:
					if i == 0:
						sine_dance(1, 1, i, rampup_count)
						sine_dance(1, 0, i, rampup_count)
					else:
						sine_dance(0, 1, i, rampup_count)
						sine_dance(0, 0, i, rampup_count)
					i += 1
					time.sleep(0.01)
		elif user_input == 'P':
			display_pos()
		elif user_input == 'C':
			display_children()



main()
_client.clean_up()
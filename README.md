# Contributors
1. Sean Fabrega (email: <fabre22s@mtholyoke.edu>)
1. Melody Su (email: <msu@mtholyoke.edu>)
# Description
This is a Python client for the Raven II Surgical Robot in the Autonomous Multibody framework developed by Adnan Munawar. Prior to this project, there was a C++ client developed by Melody Su which had similar capabilities; however, it was in C++ and was not compatible with many other capabilities within the AMBF. Much of the client translates code from the C++ client (which is largely developed from the Raven II base code) and extends certain capabilities, such as manual control and moving based on .csv file joint instructions. Using this addition, one can operate the Raven II Surgical Robot in AMBF in 4 main function: homing, sine dance, manual cartesian operation and joint operation through file input. These functions will be explained later.

# Usage
The Python client can be used on any system that AMBF can run on[. See here for details.](https://github.com/seanfabrega/ambf#:~:text=the%20corresponding%20subfolder.-,4.%20Usage%3A,-4.1%20Tested%20Platforms)

# Running the Client
1. Install ROS on your computer
2. Clone this [repository](https://github.com/MHC-RobotSimulators-Research/ambf?organization=MHC-RobotSimulators-Research&organization=MHC-RobotSimulators-Research) onto your computer
3. Follow these [instructions](https://github.com/WPI-AIM/ambf/wiki) for building ambf
4. Once you’ve installed ROS and build ambf on your computer, open your terminal
5. Run roscore


6. Open a new tab and navigate to /ambf/bin/<os> 


7. Run ./ambf\_simulator -l 3


8. Open a new tab, navigate to ~/ambf/ambf\_controller/raven2/src/raven2py


9. Run python ambf\_raven\_controller.py and follow instructions in the terminal



# Pieces of the Controller
## ambf\_raven\_def.py
This file defines important constants for mapping the motion trajectory of the Raven II, whether that be homing, sine dance, or manual control, adapted from the file ambf\_defines.cpp in the AMBF original git repository. It also contains a lot of other information about the measurements and characteristics of the Raven II, including:


|CAMERA\_COUNT|RAVEN\_JOINTS|RAVEN\_ARMS|
| :- | :- | :- |
|RAVEN\_IKSOLS|LOOP\_RATE|Eps|
|MAX\_JOINTS|MIN\_JOINTS|HOME\_JOINT|
|DANCE\_SCALE\_JOINTS|RAVEN\_JOINT\_LIMITS|RAVEN\_DH\_ALPHA|
|RAVEN\_DH\_THETA|RAVEN\_DH\_A|RAVEN\_DH\_D|
|RAVEN\_IKIN\_PARAM|RAVEN\_T\_B0|RAVEN\_T\_CB|

## ambf\_raven.py
This file defines the ambf\_raven object which contains attributes and methods for controlling the simulated raven. It connects the controller to the Raven II in the simulator upon initialization and uses values from the ambf\_raven\_def.py to help plan motion for the various control modes. 
### Variables


|\_client|homed|moved|
| :- | :- | :- |
|limited|arms|start\_jp|
|delta\_jp|home\_joints|next\_jp|
|dance\_scale\_joints|loop\_rate|raven\_joints|
|rc|rampup\_count|i|
|speed|rampup\_speed||
### Methods


|go\_home|sine\_dance|manual\_move|
| :- | :- | :- |
|move|control\_move||
#### *go\_home*
This method gradualizes the movement of the Raven II arms from the starting position to the home position, defined using the home\_joints array. The method should be called loop\_rate times and should reach the final position upon the last call of the method. If the arm is homed by the end of the final iteration, the method should return True. Otherwise it returns False. The application of this method will make more sense in the context of the ambf\_raven\_controller.py.
#### *sine\_dance*
This method slowly moves the Raven arm in a sine motion, using a sine function to move each joint. Similar to the method above, the function should be repeatedly called in a loop until the user changes control modes.
#### *manual\_move*
This used forward and inverse kinematics to create a new desired joint position array based on the change in cartesian position. In planning this method, I basically following this model:

#### *move*
This method uses a similar process to the go\_home method, but instead of HOME\_JOINTS, the method will gradually move the arm to the next desired position passed into the function. Because of the nature of the process, the movement will always take the same speed, regardless of the distance needed to travel.

## raven\_fk.py
This file contains all the methods for performing forward kinematics. There is more information in the documentation of the file for the specific processes for deriving the transformation matrices from the joint positions.
## raven\_ik.py
This file contains all the methods for performing inverse kinematics. There is more information in the documentation of the file for the specific processes for deriving the joint positions from the transformation matrix.
# ambf\_raven\_controller.py
This file is what I would consider the main function of the control program. It is the one that controls the user inputs and “talks” to the raven, for it to move. It uses an array of control switches (an array of booleans) to turn on and off certain control modes: home, sine dance, manual control and quit. In order to simultaneously wait for user input and also operate the robot, the file uses python threading and queues to communicate between the threads. For more information, look at the documentation for the file. 
## utilities.py
This file already existed in the ambf python controller for dVRK in the original github repository, however I copied it for my controller and added a few functions to mimic the methods defined in the C++ class for transformation matrices. 
# Capabilities of the Controller
## Homing
When the homing control mode is entered, both raven arms move to a predefined position defined in the ambf\_raven\_def.py file. Once it reaches that position, it will stay until another control mode is entered.
## Sine Dance
When the sine dance mode is entered, the raven arms will move in a dance pattern defined by a sine function in the ambf\_raven.py file. It will continue to move until another control mode is entered. 
## Manual Move
When the manual mode is entered, the raven arms will move in a predefined direction based on hard-coded instructions in the file. This can be hard coded by setting the values in the arrays for each x, y, z array to the distance in each direction each arm should move. It will move until that position has been assumed and then stay in that position.
## “Read File” Move
When read file mode is entered, the raven arms will move in a predefined direction based on instructions from an inputted .csv file containing joint angle data at given time stamps. Typically, this file is created from a rosbag file produced from an actual Raven II robot. This mode is especially useful for comparing movement data between the simulated and real-life Raven II surgical robot.

# Limitation of the Controller
1. No soft body support
1. No manual control in real time
1. Slow to move short distances – as of now, the client is designed to take the same amount of time to move the robot any distance in manual mode. So the speed of the movement is directly related to the distance needed to travel.





#


` `PAGE 1

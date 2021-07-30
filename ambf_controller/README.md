
## Description:
The AMBF Controller contains both a C++ and Python client which control the simulated objects in the AMBF Simulator. The Raven2 control for the C++ client is imeplemented and well tested in the AMBF Controller. Robot kinematics is implemented in the module so both cartesian and joint space commands for Raven2 is supported. The Python client supports cartesian and joint space commands as well, with multiple control modes for different predetermined patterns of movement.


## Usage:
### Building:
#### For C++ controller:
The AMBF Controller module currently supports only Linux and is built automatically with the AMBF framework. See instructions
[here](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF).
#### For Python controller:
The Python controller currently only supports Linux and can be run in the command line while simulatneously running AMBF with the Raven2 loaded and ROS.

### Setting up for Raven2 Demo:
#### For C++ controller:
Currently, a Raven2 test module is developed in the AMBF Controller. To launch
it correctly, we are going to use the custom launch_file in the raven2 folder

### Running the AMBF Controller:
#### For C++ Controller:
Having succesfully completed the steps to build, one can run the AMBF Simulator
using the following command:

```
cd ~/ambf/bin/<os>
./ambf_simulator --launch_file ../../ambf_controller/raven2/launch.yaml -l0
```
Then on a second terminal, run the AMBF Controller using the following command:

```
cd ~/ambf/bin/<os>
./raven_controller
```

Optionally, there is a convenient bash script to automate the two steps above, the script can
be execute as follows:

```
./raven_control.sh
```

#### For Python Controller:
See instructions in OnboardingDocument.pdf

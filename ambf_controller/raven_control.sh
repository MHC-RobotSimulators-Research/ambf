# The following script should automate the compilation and execution of your code
# If the .sh file is not executable, do the following:
#
# sudo chmod 700 raven_control.sh
#
# Enjoy!

printf "\n To run the code, make sure an external roscore is running.\n"
printf " To run the AMBF Controller, change the world settings in the \n"
printf " launch.yaml file to enable raven_world.yaml.\n"
printf " Choose your option below: \n"
printf "    \"p\" for python controller\n"
printf "    \"c\" for cpp controller\n"
printf "    \"ctrl-c\" to quit\n"
read user

printf "\n Run the AMBF Simulator.  (On a new terminal)"
cd ../bin/lin-x86_64/
gnome-terminal -- ./ambf_simulator -l 3
sleep 5
if [[ "$user" == "p" ]] ; then
    printf "\n Run the AMBF Python Controller. (Here!)\n"
    cd ../../ambf_controller/raven2/scripts/
    python ambf_raven_controller.py
else
    printf "\n Run the AMBF Cpp Controller. (Here!)\n"
    ./raven_controller
fi

# NEST_Control
Code for the control and simulation aspect of the NEST platform for the ATLANTIS project


#Guide to run the program

# Gazebo Simulator
roslaunch nest_control nest_launch.launch

# Control Zarco with Keyboard
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard

# Drone Simulator - Iris
catkin_ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -L FADEUP --console --map

#Run QGroundControl 
cd catkin_ws
./QGroundControl.AppImage

#Run NEST platform simulation
rosrun nest_control nest_spawn

#Run Follow node 
rosrun nest_control nest_control

# RESET BATTERY SIMULATOR (IN CASE THEY RUN OUT)
rosservice call /mavros/cmd/command "{broadcast: false, command: 42651, confirmation: 0, param1: -1.0, param2: 100.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"


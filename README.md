# NEST_Control
Code for the control and simulation aspect of the NEST platform for the ATLANTIS project


## Guide to run the program

### Gazebo Simulator
```bash
roslaunch nest_control nest_launch.launch
```
In a separate terminal run: 
### Control zarco with keyboard
```bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
```
In another terminal run:
### Drone Simulator - IRIS
```bash
catkin_ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -L FADEUP --console --map
```
In any terminal:
### Run QGroundControl
```bash
cd catkin_ws
./QGroundControl.AppImage
```
Arm and takeoff the NEST

In another terminal:
### Run NEST platform simulation
```bash
rosrun nest_control nest_spawn
```
Create another terminal and run this code:
```bash
roslaunch mavros_extras kbteleop.launch
```
Click in number 0 on the numpad until the NEST reaches the water.


### Run Follow node 
```bash
rosrun nest_control nest_control
```

### RESET BATTERY SIMULATOR (IN CASE THEY RUN OUT)
```bash
rosservice call /mavros/cmd/command "{broadcast: false, command: 42651, confirmation: 0, param1: -1.0, param2: 100.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

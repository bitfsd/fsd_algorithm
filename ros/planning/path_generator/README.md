# Path Generator

This ROS package is an open source version of planning package from BITFSD, which is built for generating the reference path.  

There are three methods in the path generator, each matched with a specific competition mission. 

## Important Dependencies

* catkin_tools
* ROS

## ROS topic

### Acceleration Mission

* **Subscriber**
  * /estimation/slam/state
  * /planning/end_point
* **Publisher**
  * /control/pure_pursuit/control_command

### Skidpad Mission

* **Subscriber**
  * /estimation/slam/state
  * /transform_matrix
* **Publisher**
  * /control/pure_pursuit/control_command

### **Trackdrive Mission**

* **Subscriber**
  * /map
  * /estimation/slam/state
* **Publisher**
  * /control/pure_pursuit/control_command

## Step

### Prepare

```bash
# Build your workspace
catkin build

# Source environment
source devel/setup.bash
```

### Run simulation

Please refer to this [Repository](https://github.com/bitfsd/fssim) .

### Launch

```bash
# For Trackdrive Mission
roslaunch path_generator trackdrive.launch

# For Acceleration Mission
roslaunch path_generator acceleration.launch 	

# For Skidpad Mission
roslaunch path_generator skidpad.launch						
```

## Parameters

In the `./config` folder, we provide three `.yaml` files. Each file contains different parameters for the three FSAC competitions respectively.

* **Subscriber:** List the ros topics which the code will subscribe.
* **Publisher:** List the ros topics which the code wil publish.
* **Trajectory params:** Describe the parameters related to the generation of trajectory.
* **mission:** You can choose the mission here.
* **simulation:** If you are running in [simulator](https://github.com/bitfsd/fssim), make sure this parameter is true, or it will be a car crash in skidpad match.

## Note:

* Topic `/transform_matrix` comes from [skidpad detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/skidpad_detector).
* Topic `/planning/end_point` comes form [line detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/line_detector).
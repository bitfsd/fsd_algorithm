# Controller

This ROS package is an open source version of control package from BITFSD.

We implement two controllers for the three FSAC competitions, pure pursuit controller and model predictive controller (MPC).

## Important Dependencies

* catkin_tools
* CppAD
* IPOPT
* ROS

## Install

```bash
cd ~/fsd_algorithm/ros/control/script

# install build tools
sudo apt-get install python-catkin-tools

# install IPOPT solver
sudo bash install_ipopt.bash

# install Algorithmic Differentiation tools
sudo bash install_cppad.bash
```

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

### Launch controller

```bash
# For Trackdrive Mission
roslaunch control trackdrive.launch

# For Acceleration Mission
roslaunch control acceleration.launch 	

# For Skidpad Mission
roslaunch control skidpad.launch						
```

## Params

In the `./config` folder, we provide three `.yaml` files. Each file contains different parameters for the three FSAC competitions respectively.

* **Subscriber:** List the ros topics which the code will subscribe.
* **Publisher:** List the ros topics which the code wil publish.
* **MPC params:** Describe the MPC parameters.
* **Pure Pursuit params**: Describe the pure pursuit parameters.
* **controller:** You can choose the controller here.
* **simulation:** If you are running in [simulator](https://github.com/bitfsd/fssim), make sure this parameter is true, or it will be a car crash in skidpad match.

## Note:

* Remember to adjust the `car_length` in the `.yaml` file, once you want to run different racecar.
* If you want to be faster, you can change the  `desire_vel` in `.yaml` file.
* Topic `/transform_matrix` comes from [skidpad detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/skidpad_detector).
* Topic `/planning/end_point` comes form [line detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/line_detector).


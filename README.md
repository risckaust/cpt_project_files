# Cooperative Payload Transportation Project (CPT)
*Daniel Williams, May - September 2018*

## Matlab Simulation
### Instructions for Usage
1. Ensure that MATLAB R2016 or higher is installed
2. The files can be run from the directory `./MATLAB`

### Description of Files

* `cpt_toplevel.m`: top-level file for customizing parameter values, running simulations and plotting output. This is generally the only file you will need to modify. Modifiable parameters include:
    * times: start, end, and transition points
    * cable length and tension (assumed to be _mg/2_)
    * payload geometry, mass and moment of inertia
    * PID gains for each agent's x-plane and y-plane motion
    * position setpoints
    * maximum velocity
    * initial values for each state variable
* `cpt_startupmode.m`: holds the system of differential equations describing the startup flight mode of the simulation
* `cpt_gatekeeper.m`:
* `cpt_carrymode.m`: holds the system of differential equations describing the carrying flight mode of the simulation
* `cpt_dropoffmode.m`: holds the system of differential equations describing the drop-off flight mode of the simulation
* `cpt\_sat_o.m`: limits output in order to remain within user-defined velocity constraints
* `cpt\_sat_i.m`: a velocity limiter that uses a built-in interpolator with the user-defined position setpoints

### Outstanding Bugs
* Offset from zero degrees for θ1 and θ2 during dropoff mode (23 September 2018)
* ϕ is not giving sound output, issue not solved by using `ode15s` instead of `ode45` (23 September 2018)

### Changelog
* **23 September 2018**: renamed files, *first commit to the new remote*

## ROS-Gazebo-MAVROS-SITL Simulation
### Instructions for Usage

1. Ensure you have followed the instructions for installing prerequisites for the software-in-the-loop joystick flight package (see [here](https://risc.readthedocs.io/1-flight-gazebo.html))
2. Copy the folder `cpt_sitl_pkg` to `~/catkin_ws/src/`, then build and source the catkin workspace
3. Open a terminal shell and do the following:
    roscd cpt_sitl_pkg
    cd ./build
    cmake ../
    make
For a demonstration of the **angle-measuring plugin**, an example world with a pendulum is included in `/worlds/pendulum.world`. 
1. In a terminal shell run the following:
    roscore
2. Open a new terminal shell and run the following:
    roscd cpt_sitl_pkg
    gazebo ./worlds/pendulum.world --verbose
3. To see the pendulum in action, right click the model and set it to **Wireframe**
4. In Gazebo, bring up the Topic Visualization panel with `Ctrl+T` and double-click on `/Pendulum/joint_angle` to see the published angle messages.
5. Open a new terminal shell and run `rostopic echo /Pendulum/joint_angle` to observe the published angle messages in ROS

A demonstration of the **customized model and world** for the cooperative payload transportation project is also available. Simply run `roslaunch cpt_sitl_pkg multiuav_demo.launch`.

### Description of Files
* `/build`: directory for building the angle-measuring plugin
* `/launch`: contains launch files for the angle-measuring plugin demonstration and the CPT flight demonstration
* `/models`: contains the modified 3DR Iris model, with two UAVs, a cylindrical payload and cables
* `/scripts`: contains a Python script that sends position setpoints for each agent to MAVROS for onboard execution
* `/worlds`: contains the example world for the angle-measuring plugin demonstration
* `/joint_angle_plugin.cc`: C++ source code for angle-measuring plugin
* `/CMakeLists.txt`: required for CMake
* `/package.xml`: required for ROS

### Outstanding Bugs

* Custom launch file for pendulum demonstration does not launch the plugin (25 September 2018)
* Gazebo requires high GPU capacity to run smoothly, therefore low real-time factor and unsound IMU readings for the leader agent UAV0 (25 September 2018)

### Changelog
* **25 September 2018**: merged angle-measuring plugin files into the cpt\_sitl_pkg ROS package, *first commit to the new remote*
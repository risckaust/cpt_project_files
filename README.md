#Cooperative Payload Transportation Project
*Daniel Williams, May - September 2018*

## Matlab Simulation Files


* **cpt_toplevel.m**: top-level file for customizing parameter values, running simulations and plotting output. This is generally the only file you will need to modify. Modifiable parameters include:
 * times: start, end, and transition points
 * cable length and tension (assumed to be _mg/2_)
 * payload geometry, mass and moment of inertia
 * PID gains for each agent's x-plane and y-plane motion
 * position setpoints
 * maximum velocity
 * initial values for each state variable
* **cpt_startupmode.m**: holds the system of differential equations describing the startup flight mode of the simulation
* **cpt_gatekeeper.m**:
* **cpt_carrymode.m**: holds the system of differential equations describing the carrying flight mode of the simulation
* **cpt_dropoffmode.m**: holds the system of differential equations describing the drop-off flight mode of the simulation
* **cpt\_sat_o.m**: limits output in order to remain within user-defined velocity constraints
* **cpt\_sat_i.m**: a velocity limiter that uses a built-in interpolator with the user-defined position setpoints

## Materials for ROS-Gazebo-MAVROS-SITL Simulation
# mcl
Webots ROS2 Monte Carlo Localizer

## Description
The Monte Carlo Localizer package demonstrates the localization using particle filter.
Given the map of the environment, the package outputs its best pose estimate.
In the example, the robot used is e-puck running in the Webots simulator.

## Build
The following instructions assume that a ROS2 workspace is already set up.
```commandline
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --packages-select mcl
. install/setup.bash
```

## Execution
After building and installing the package, you can launch the simulation.

### Running Simple Mission
```commandline
ros2 launch mcl mcl_launch.py rviz:=true mission_time:=10
```
mission_time is the time for the robot to bounce in minute.

### Parameters
You can change the parameters for the motion model, sensor model, or the mcl.

#### Motion Model Parameters
You can specify alpha1, alpha2, alpha3, and alpha1 which will affect the noise generation for rotational and linear speeds. 
In this package, we are using Odometry Motion Model

#### Sensor Model Parameters
For sensor model, the default is using the Likelihood Fields, so you can configure the standard deviation to set the fields.
If you choose to use the Beam Model, there are no parameters needed.

#### MCL Parameters
adaptive: using adaptive particle filter (the number of particles vary based on needs)
likelihood_model: to choose the sensor model, true: likelihood fields, false: beam model
num_of_particles: number of particles

For adaptive Particle Filter, there are more parameters that you can change to configure the number of effective particles:
epsilon
upper_quantile
min_number_of_particles
max_number_of_particles

## Dependencies
Webots ROS2 package

## Documentation
If you're interested in understanding the details, please read my post [here](https://towardsdatascience.com/particle-filter-localization-with-webots-and-ros2-619ecf0c5f08)

## Issues
Please report issues if you found bugs or raise a Pull Request.

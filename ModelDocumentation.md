# Model Documentation

## Introduction and Overall Code Structure

The path planner implemented for this project contains essentially two classes,
the VehicleController class and the TrajectoryGenerator class. All general
purpose utility and helper functions have been implemented/moved into utils.cpp
to reduce clutter in main.cpp.

While the TrajectoryGenerator class is responsible for the generation of smooth,
drivable trajectories for the ego vehicle, the VehicleController class
essentially governs the behavior of the ego vehicle and decides whether to
change lanes, keep cruising near the speed limit, or slow down because the
traffic ahead moves at a lower speed. Once the vehicle controller has decided
what to do, it calls the trajectory generator to generate a suitable
trajectory for the desired behavior.

In the following two sections, I will briefly outline the concepts
and rational behind the implemented path planner.

### Vehicle Controller
The vehicle controller contains a relatively straight forward behavior planner to
decide whether to keep driving in the current lane or to change lanes.
The default behavior, i.e., if the ego vehicle is not obstructed by traffic, is
to keep cruising in the current lane with a speed close to the speed limit.

The vehicle controller constantly checks if the road ahead is obstructed by traffic.
If that is the case, there are three options. One, keep driving in the current lane
and slow down. Two, perform a lane change to the left, or three perform a lane change
to the right. The vehicle controller checks whether a lane change to the left or to the right
would be possible, by verifying that the ego vehicle would not veer of the road in
case a lane change is performed. In addition the controller checks whether the ego
vehicle would collide with, or at least come very close to  
other cars if a lane change were performed.

If the vehicle controller determines that a lane change is safely possible, the
controller asks the trajectory generator to compute a smooth drivable trajectory
which reflects the desired lane change maneuver. Otherwise the speed of the car is adjusted
according to the distance to the car ahead and its speed and the trajectory generator is
asked to generate a "keep-lane" trajectory.

In addition, the vehicle controller has an emergency break maneuver which is triggered
if another car makes a lane change into the lane of the ego vehicle right in front of
the ego vehicle.

### Trajectory Generator
The trajectory generator gets passed the previous, yet unused points, of the trajectory,
the current state of the vehicle, and the target lane to which a trajectory
should be generated. The trajectory generator now generates a smooth path by first creating
a spline which interpolates between the last points of the previous path and some way points
that are in the target lane and further down the road. By adjusting the distance between these
generated way points and the ego vehicle, the maximum jerk can be controlled.

Subsequently, the interpolating spline is then sampled to generate a series of points
that represent the desired trajectory and which are then fed to the simulator.

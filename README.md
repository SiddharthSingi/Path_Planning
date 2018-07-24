# Path_Planning
Planning the path of an autonomous car on a simulator.


## Goal
The goal of my project is to use the sensor fusion data to generate a safe and viable path for the vehicle to follow on a highway road. The vehicle must be able to:
* Stay in the same lane.
* Follow a smooth trajectory along the road.
* Change Lanes on a highway.
* Not excced the maximum speed limit.
* Not excced the maximuma acceleration limit.
* Not exceed the maximum jerk limit.
* Travel at least 4.32 miles without colliding with other vehicles on the highway.
* Stay in the fastest Lane to reach its goal in the least amount of time.
* Incorporate safety as well as efficiency into the vehicle.


## Implementation

1. ### Spline Usage to genrate a path
In the main.cpp file I used getXY() methods to convert Frenet Coordinates to X, Y coordinates, I used 5 points to generate a spline and then extrapolated points from the spline to give to the simulator. This is done in the lines(  to  ), the five points I used were the second last point in the previous path, the last point in the previous path, and points 45m, 60m, and 75m from the current s poistion of the car. All these points are translated to the last point of the previous path and rotated to the ref_yaw. The notation ref_x, ref_y, ref_s, etc are all used to denote the variables at the end point of the previous path.

2. ### Changing Lanes
To change lanes I considered two things, first is a lane change possible, and if lane changes are possible in both the right and left lane, then to shift to the lane which has a higher traffic speed. To implement this I created a Class Lane, where a new Lane object is created for every lane. The Lane Class essentially has two methods `bool LaneShift` and `double LaneSpeed`. LaneShift tells whether a lane shift in the particular lane is possible or not, and LaneSpeed gives the maximum speed the car can travel at in that particular lane.

3. ### Speeding Up and Slowing Down



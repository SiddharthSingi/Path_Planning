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
In the main.cpp file I used getXY() methods to convert Frenet Coordinates to X, Y coordinates, I used 5 points to generate a spline and then extrapolated points from the spline to give to the simulator. This is done in the lines(983 to 1057 ), the five points I used were the second last point in the previous path, the last point in the previous path, and points 45m, 60m, and 75m from the current s poistion of the car. All these points are translated to the last point of the previous path and rotated to the ref_yaw. The notation ref_x, ref_y, ref_s, etc are all used to denote the variables at the end point of the previous path.

2. ### Changing Lanes
To change lanes I considered two things, first is a lane change possible, and if lane changes are possible in both the right and left lane, then to shift to the lane which has a higher traffic speed. To implement this I created a Class Lane, where a new Lane object is created for every lane. The Lane Class essentially has two methods `bool LaneShift` and `double LaneSpeed`. LaneShift tells whether a lane shift in the particular lane is possible or not, and LaneSpeed gives the maximum speed the car can travel at in that particular lane.

3. ### Speeding Up and Slowing Down
To speed up or slow down I have first divided the target_dist into N points using `ref_vel`, Then I used the x-component of these divisions to extrapolate points on the spline. The `target_vel` variable is used to either accelerate, deccelerate or maintain speed. The target_vel is initially set to 49 and if there are any vehicles in front of us then, the target velocity is reduced to the value of that vehicle.
After every point that is added the ref_vel is either increased or decreased by an acceleration calculated in such a way so as for the ref_vel to reach the target_vel at a dist of 25m in front of the target_vehicle. Lines(1093-1105)

4. ### Safety
To avoid rapid jerks in speed of the vehicle I have incorporated a maintain speed boolean this ensure that vehicle does not accelerate or deccelerate too often, However, at times this causes our car to have a faster velocity then the car in front of us, So to avoid a collision with the vehicle in front, I have programmed our car to accelerate at its maximum allowed value every time the car comes within a distance of 5m from the front vehicle.


## Possible Improvements:
* The car sometimes changes lanes too often.
* Some more multidirectional safety features can be integrated into the car to avoid any collision at all.
* The possibility of a lane change should change depending upon the speed the vehicle is travelling at.
* Currently the car maintains speed for a very brief time period, which I amsure could definitely be improved.

You can watch the video of my path planning implementation on [YouTube](https://youtu.be/Z6ye_7wpOEk)

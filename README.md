# Self-Driving Car Engineer Nanodegree Program
# *CarND-Path-Planning-Project*

## Introduction

In this project, a car has to be navigated around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit considering lane change to keep speed and checking the traffic around to avoid collisions. Additionally, the car should stay in the marked road lines and should avoid accelerations greater than 10 m/sˆ2 and jerk over 10 m/sˆ3. The project is successful if the car completes the full round (6946m = 4.32 miles) without violating above given rules.


<p align="center">
<img src="Images/Photo3.jpeg" height=350 />
</p>


**Result**:
* Example incl. lane change [YouTube video] (https://www.youtube.com/watch?v=csJzQ6U3IFc)
* Result showing completion of task [YouTube video] (https://www.youtube.com/watch?v=3k7RCY44_S8)


## Path Planning

The code for the path planning mainly consists of the three blocks **prediction** (main.cpp, l. 308-476), **behavioral control** (main.cpp, l. 479-692), and **trajectory** generation (main.cpp, l. 695-786).

For **prediction**, each of the foreign car's position at the end of the trajectory is calculated and afterwards it is checked if it appears in one of the following regions as the nearest car:
* red_range: The foreign car is too close to the vehicle. Main action is to slow down the vehicle.
* mid_range: In this region, the velocity of the car is adopted to the foreign car's velocity and it is checked if the car can change the lane.
* far_range: This is to prepare the car for a lane change.

Additionally, it is checked whether a foreign car is in the far_range on the right lane (no change to the right lane), if the car is in the middle of the lane (end condition for changing a lane) and if the distance between the nearest two foreign cars on the left or right lane is large enough (ready_4_change).

Based upon the region information, the following states take their actions for **behavioral control**:
* keep_lane: If a car is in front of the car, it either slows down or adopts the velocity and tries to change lanes. The left lane is preferred. Otherwise, if nothing is in front of the car and the right lane is free, it turns over to the right lane.
* prep_4_left_lane: According to the distance to the foreign car in the front, the velocity will either be slowed down or adopted. If the distance of the two nearest foreign cars on the left lane is big enough, the lane change will be started.
* prep_4_right_lane: The same as above but for the right lane
* change_2_left_lane: As long as the left lane is clear for a change, the velocity control will be continued until the car is in the target lane. If the left lane is not clear any more, the status goes to the preparation stage for the right lane change.
* change_2_right_lane: Same as above but for the right lane. If the right lane is not clear anymore, the status goes to the preparation stage for the left lane change.

The **trajectory** calculation is based upon splines. The method is taken over from the lecture. For the spline calculation, five breakpoints ((x,y)-tupel) are determined, the two last ones from previous trajectory calculations and three on the future way. Before calculating the spline parameter, the breakpoints are transformed into the car's coordinate system. With the help of the determined target velocity during behavioral control, the number of points on a given length of the trajectory to a target distance is calculated. The according number of spline points are calculated in the car's coordinate system and transformed back to xy-map-coordinates before they will be added to the previous trajectory points which haven't been reached by the car. All of them build the new trajectory.






---

# **Original Udacity README**


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


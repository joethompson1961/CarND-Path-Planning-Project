# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Reflection
This project depends on 2 basic elements:  Behavior Planning and Trajectory Planning.

Behavioral planning uses sensor input to make high level decisions about the actions of the car while the trajectory planning derives drivable paths that execute the intentions of the behavioral planning.

#### Behavioral Planning
The behavioral planning state machine (see main.cpp::308-450) isn't implemented with a formal structured FSM design. This makes the state machine more compact and efficient because of the use of code blocks that are applicable regardless of the state, but it can also makes it harder to understand and follow. If the behavioral state machine needed to be made more complex it would be wise to use a more structured design approach.  But be assured, the behavioral states described here are fully implemented as described.

There are two outputs of the behavioral planner that the trajectory planner depends on:
* The desired traffic lane to be in.
* The speed at which to travel.
 
The behavioral planning is implemented with a state machine comprised of 4 states:
* __Cruising__
    * The speed of the vehicle is adjusted to go as fast as possible while obeying the speed limit and not violating the limits on maximimum jerk and acceleration. A simple proportional control is used to adjust the rate of accleration based on the difference between current speed and target speed. The greater the difference, the greater the acceleration but never to exceed the maximum jerk limit of the project (see main.cpp::364-368).
    * The vehicle looks ahead in it's lane for slow cars in front (see cpp.main::322-325).
    * When a slow car is detected in front of ego-car and in the same lane and it's predicted future position is within the predicted future safe zone 50 meters in front of ego-car it switches to __Following__ state (see main.ccp::339-349).
* __Following__
    * The vehicle makes adjustments to speed to match the speed of the vehicle in front and to stay a safe distance behind it. A proportional control is used to adjust the rate of accleration based on the differnce in speed between the vehicles and the gap distance to the car in front (see main.ccp::359-363).
    * The car searches lanes to the left (see main.ccp::386-410) and right (see main.ccp::412-436) and, when a lane is deemed clear, it sets the new target lane and changes to either __LaneChangeLeft__ or __LaneChangeRight__ state (see main.ccp::438-448). 
* __LaneChangeLeft__ / __LaneChangeRight__
    * While changing lanes the vehicle looks ahead for slow cars in front in both the orignal lane and the target lane and adjusts speed to maintain a safe distance from the closest car in front as done in __Following__ (see main.ccp::325-326).
    * When the vehicle is determined to be completely within the target lane the car switches to __Cruising__ state (see main.ccp::371-380).
   
#### Trajectory Planning
The trajectory planner (see main.cpp::452-533) is implemented based mostly on the Arron's walk through implementation with some small variations.

This approach to trajectory planning uses a spline tool (see main.cpp::510:511) to calculate a single path based on the target speed and lane inputs from the behavioral planner.  Working with the behavioral planner it ensures that the derived trajectory satisfies the maximum jerk and maximum acceleration limits of the project.

To produce a new trajectory the planner initializes the spline with the remaining unconsumed waypoints from the previously generated trajectory (see main.cpp::466-490) and then extends the spline to points well ahead of the car and within the lane selected by the behavioral planner (see main.cpp::492-500). This approach ensures smooth transitions from the previous trajectory to the new goals in reponse to changes in behavior state, e.g. switching from following to lane change requires a trajectory that avoids sudden jerks as the vehicle's path is adjusted to switch lanes.

With the spline tool initialize it's then very straight forward to generate x,y points for the new trajectory by calculating future x points based on target velocity and lane and then using the spline tool to produce corresponding y points (see main.cpp::520-535).

This approach for trajectory works well for this project but, for real driving, a single trajectory that aims for the center of the target lane and executes lane changes always using exactly the same style won't always produce the best outcome and may not be flexible enough to handle the numerous corner cases encountered in real driving. An alternative approach that evaluates numerous possible paths and selects the best path based on a cost function is preferred and if I had more time this is how I would implement it.  

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

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


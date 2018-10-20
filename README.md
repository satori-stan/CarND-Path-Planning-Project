# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Simulator.

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab] (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Goals

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit. From the car's subsystems, we will be provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway.

A successful implementation requires:

* Code that compiles correctly with CMake and Make
* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
* The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
* Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


### Here I will consider each point individually and explain how I addressed them in my implementation

#### Compilation

The starter code provided for the project was already meant to compile correctly in Linux, Mac and Windows; although compilation in Windows required installing Windows 10 Bash on Ubuntu or using Docker to create a virtual environment.

In previous projects for this degree, I had chosen to develop natively in Windows. This presented some issues, but after the first project of the second term (Extended Kalman Filter) I had an environment that would allow me to build most other projects.

There are a couple of changes in the CMakeLists file to include all the source files and for native compilation in Windows. I also included some Macros to allow compilation against either uWebSockets version v0.14.x or previous versions, which caused issues in the last project review.

#### The car should try to go as close as possible to the 50 MPH speed limit.

The base of the logic is a FSM (finite state machine) with four states: start-up, stay on lane, change lane right and change lane left. 

The start state is only ever used when the simulation begins and the car is stopped. The car immediately switches to the "stay on lane" state. This state and the "change lane" states are selected based on whether they are possible within the environment's limitations (the car can't change to a lane with oncoming traffic or that is outside the road) and they represent the best state to achieve all the requirements of the goal.

The environment's limitations are quite clear (only those two stated above), but to choose the best state to achieve all the requirement's goals there are a number of competing concerns (go fast, avoid collisions, stay in the lane, etc.). To choose the best state means to balance the importance of each of these concerns, which is done using cost functions.

For each possible state, the trajectory generator will propose at least a couple of options (with more or less acceleration) and the path planner will choose the one with the least cost. The next state is then based on the state that the winning trajectory represents.

The cost function to push the car to drive as close as possible to the speed limit, penalizes lower speeds. The function is different than those used in the lessons: the logistic function meant to keep the cost base between 0 and 1 was great for values that were far from one another but as the ratio between the current speed and the maximum speed approached the limits, the difference was less discernible. The function used is:

    max((target_speed - current_speed) / (current_speed + 1), 0) / target_speed

A differential between the target speed and the current speed is divided by the current speed. This arrangement is because the cost is inversely proportional to the current speed. Adding 1 to the current speed in the divisor prevents a division by zero and while it reduces the usable range of values, it still provides good resolution.

I max out the cost at zero (i.e. there is no gain in going over the target speed). The final division by the target speed keeps the cost base at or below 1 (given the car doesn't go in reverse!).

The cost "gain" of this "efficiency" measurement is (in its current state) the largest one (250).

In order to get the car moving, there is an extra cost of 100 if the speed is zero, and 50 extra if the speed is negative (reverse).

Speed gain is not the only concern though: changing lanes just to get a speed boost (as when done in a curve) is discouraged if the road is clear in front of us.

#### The car should avoid hitting other cars

There are a lot of variables to consider to avoid hitting other cars. For each of the proposed trajectories, a check is done against all the known adversaries (from the sensor fusion) to identify the closest in front of the car in its current location (both in the current and target lanes), the closest behind the car in its current location in the target lane, and the closest in front and behind the car in its final position in the target lane. The velocities of the adversaries are also taken into account.

The considerations are:
* The distance between the car and the adversary in front given the final position should be as much as possible greater than a safety distance (30m).
* The time to hit a car that is in front of us given the current and target positions in the target lane should always be over 10 seconds, regardless of the distance. This is calculated with the difference between the car's and adversaries' velocities and the distance between one another.
* The time for a car to hit us from behind given the current and target positions in the target lane should always be over 10 seconds, regardless of the distance.
* An adversary should never be within the vehicle radius in front of behind our position (current or target).
* The same adversary shouldn't be in front of us in our current position or behind us in the target position in the target lane.

Trying to change lane and coming up short of the target lane is also penalized.

Cost gains are as varied as 100 and 2, given the number of scenarios analyzed. To balance the competing concerns, key variables of each proposed trajectory are logged, as are the vehicle positions and are stored to make sure that gain modifications to get a specific outcome don't adversely affect the results expected in other scenarios.

The scenarios, formulas and gains are documented in the included [Excel file](./Cases.xlsm).

#### The car should be able to make one complete loop

Provided is an animation of the last few meters of the track. The indicators show that there were no incidents for the full length.

![The car completing one loop around the track](./final_moments.gif)

It is of course entirely possible to complete more than one loop, but the program is not prepared yet to do so.

#### The car shouldn't experience total acceleration or jerk with a magnitude over 10

This is probably one of the trickiest points, one that I am not sure is entirely solved. There are costs associated with going over the maximum speed, acceleration and jerk values, but having used the spline library these calculations proved to be extremely hard. Every now and again, the car will experience jerk and acceleration values that are too high.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Relevant data

#### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually it�s not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this it�s a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

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


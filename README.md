# PID Controller
---

## Project Overview
Control in a self driving car is how we use the steering, throttle, and brakes to move a car where it needs to go. The following project is a C++ implementation of a PID controller to maneuver a vehicle around a simulated racetrack. 

The simulator provides the cross track error (CTE), which is a measure of how far the vehicle is from the center of the lane. The PID controller is responsible for getting the vehicle to the center of the lane as quickly as possible with minimal overshoot and steady state error. 

The simulator also provides the vehicle velocity in miles per hour (MPH). A second speed controller was implemented to allow the car to reach the desired speed of 30 MPH quickly, and without overshoot. 

The speed controller was given throttle limits which were used to saturate the controller and the steering controller was given steering angle limits to saturate the controller. Both of these bounds were set to model a realistic car. 

The following GIF shows the angle and speed controllers both working on the simulated self driving car as it successfully travels around the track.

![](./images/runExample.gif)

## Hyperparameter Tuning

The following control block diagram was used when modeling the PID controller in C++. The control block diagram explains how the output from the system is related to the set input and the error signal. In this system, the output was the vehicle behavior - steering angle or speed. The setpoint was the desired behavior - middle of road or 30 MPH. The process was the dynamics of the car, which are modeled in the simulation. 

![](./images/pid.png)


The proportional gain was needed to reduce the error in the signal at a faster rate. Proportional gain can be thought of as how hard you push the controller to get the system to the desired state. Directly increasing the proportional gain decreases the rise time of the system at the cost of increasing the potentail overshoot. In both the steering controller and speed controller increasing the proportional gain was needed to increase the system response, which was expected.

The integral gain was needed to reduce the steady state error in the system. The integral gain needed to be extremely small in this system since little steady state error was seen while driving. 

The derivative gain was needed to reduce the overshoot in the system. Adding derivative gain helps improve the stability of the system and decreases the overshoot - at the cost of potentailly decreasing the rise time. Derivative gain was needed in the system to stop the car from missing the center and continually oscilating back and forth. In some cases, it was seen that without derivative gain, the steering would be unstable. 

The final hyperparameters were set using the twiddle method. In the twiddle method the P, I, D parameters are increased and decreased seperately and the overall effects these increases and decreases have are scaled until a local optimum of parameters are found. The twiddle method was run for the speed controller and the steering controller. It was found that some additonal tuning of the starting twiddle parameters could help the controller perform a better estimate. For example, the change in the integral tuning from twiddle needed to be scaled down signifigantly since little steady state error occured in the system, which is most likely due to the simulation. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Running the Code
Download the simulator and open it. In the main menu screen select Project 4: PID Controller. The simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Run the simulator to see how the car travels around the track.


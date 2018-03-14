[//]: # (Image References)
[image1]: ./img/finite_states.png "Finite States"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Introduction

This code implements path controller to a vehicle over a simulation scenario. The major goal is to keep the car driving over at least 4.32 miles without any incident.

The controller is responsible to keep the car in the track, driving safety and smooth, respecting rules (maximum speed, drive inside a lane) without proceeding any dangerous maneuvers that can cause a collision with other vehicles. 
The controller is also responsible to drive smooth and comfortable to the passengers, without high acceleration, velocity or jerk.

## Path Planning

To define the behaviour of the vehicle in the track, it's used a class "Path Planning", responsible to keep the car in the track, choose the best lane, define a safety velocity and if is safe to change lanes.

To do that, the solution is created over a Finite State Machine, that helps the system to understand which situation is happening at the moment. 

When the situation is known, the controller can decide how to drive, the moment to execute a manoeuvre, increase or decrease velocity.

### Finite State Machine

Finite State Machine was implemented in the class ***FSMachine***, inside ***fsm*** namespace. *FSMachine* control transition over the possible states (created as an enum structure ***finite_states***).

![Finite states and possible transitions][image1]

Differently from course classes, I decided to join "Prepare to Change Lane Left" and "Prepare to Change Lane Right" into just one state, that can decide to change left or right if it's is possible. This solution was better once I don't prepare the car to change lane, going to the side of the lane. This state only indicates that's the car is looking for an opportunity to pass the car in front, or decrease velocity if there are no change opportunities.

### Path Control

Path control is the responsibility of ***PathPlanning*** class. This class control the current car. It has a finite state machine *FSMachine*, described above, that indicates the current state of the vehicle. The class also contains information about the vehicle as its velocity, current lane and sensor information (about other vehicles).

The class get information about all environment and decide what is the best decision to make - keep lane, prepare to change lane, change lane, increase or decrease velocity. This class should control the movements of the car, keeping safety. Once a maneuver is completed, ***PathPlanning*** class send a transition order to finite state machine controller, to keep information about the vehicle updated.

### Possible manoeuvres

After *Ready* state, the go to the *Keep Lane* state. In this state, the vehicle tries to move in the maximum allowed velocity for the road/lane. It keeps in the lane if there is no maneuver predicted.

If there's a car in front of the vehicle, moving at a lower velocity than car's current speed, the path planning control tries to change lane. The following operations is done by the algorithm:
1. change the finite state to *Prepare to change lane*;
1. if a change to the left lane is possible, change state to *Change Left* and proceed maneuver;
1. else, try to turn into right lane;
   1. if change to the right lane is possible, change state to *Change Right* and proceed maneuver;
   1. else, reduce velocity to the same as the front car and wait until it's possible to change lane (back to item 2);

Completing the described algorithm, before proceed any maneuver, the algorithm evaluates if change is a good option. Change lane to other where the car is near the current lead car is not a good option.

After proceeding a maneuver, the algorithm returns to the state *Keep Lane* and go tries again to run near the maximum velocity available for the lane. The algorithm then starts again.

[//]: # (Image References)
[image1]: ./img/finite_states.png "Finite States"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Introduction

This codes implements path controlller to a vehicle over a simulation scenary. The major goal is keep the car driving over at least 4.32 miles without any incident.

Controller is responsible to keep the car in the track, driving safety and smooth, respecting rules (maximum speed, drive inside a lane) without proceed any dangerous maneuver that can causes colision with other vehicles. 
Controller is also responsible to drive smooth and confortable to the passengers, without high acceleration, velocity or jerk.

## Path Planning

To define the behaviour of the vehicle in the track, it's used a class "Path Planning", responsible to keep the car in the track, choose the best lane, define a safety velocity and if is safety to change lanes.

To do that, the solution is created over a Finite State Machine, that helps the system to understand which situation is happening in the moment. 

When the situation is known, controller can decide how to drive, the moment to execute a maneuver, increase or decrease velocity.

### Finite State Machine

Finite State Machine was implemented in the class ***FSMachine***, inside ***fsm*** namespace. *FSMachine* control transition over the possible states (created as an enum structure ***finite_states***).

![Finite states and possible transitions][image1]

Differently from course classes, I decided to join "Prepare to Change Lane Left" and "Prepare to Change Lane Right" into just one state, that can decide to change left or right, if it's is possible. This solutions was better once I don't prepare the car to change lane, going to the side of the lane. This state only indicates that's the car is looking for an oportunity to pass the car in front, or decrease velocity if there's no change oportunities.

### Path Control

Path control is responsibility of ***PathPlanning*** class. This class control the current car. It has a finite state machine *FSMachine*, described above, that indicates the current state of the vehicle. The class also contains information about the vehicle as its velocity, current lane and sensor information (about other vehicles).

The class get information about all environment and decide what is the best decision to make - keep lane, prepare to change lane, change lane, increase or decrease velocity. This class should control the moviments of the car, keeping safety. Once a maneuver is completed, ***PathPlanning*** class send a transition order to finite state machine controller, to keep information about the vehicle updated.

### Keep Lane and Adjust velocity

### Change Lane
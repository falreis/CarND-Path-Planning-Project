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

### Path Control

### Keep Lane and Adjust velocity

### Change Lane

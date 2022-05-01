# Path planning
This project implements a basic functionality to demonstrate a simple path planning algorithm. A "robot" object make steps with a predefined distance to move along a path defined throug a number of waypoints. It reports when a waypoint or end is reached and when an step is completed.
## Implementation
Three classes and a function are declared in `pathPlanning.h`
## Point
Store cartesian coordinates of the point and implements functions to calculate distance to another point and to generate a new point separated a predefined distance following the direction towards other point.
## Pose
A class to define the spatia status of the robot. A 3X3 rotation matrix is used to identify the orientation and complement the position. Also a function to reorientate X axis towards a point is generated.
### Reorientation
X vector of the coordinated system is considered as the forward movement of the robot. After aligning it with the direction of movement, Z vector (pointing upwards) and Y (point toward the left) are calculated.
## Robot
It is positioned by default at point 0,0,0. A path can be defined into it adding as waypoints as needed and a command to move following the path with the requirements defined is available.
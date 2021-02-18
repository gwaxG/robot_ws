# Control
***
### Description
The package "Control" provides Jaguar robot control both in real world and simulation.
Hereafter, the robot state is a vector of linear velocity, angular velocity, arm joints angles, front and rear flippers angles.
The exended (initial) state is when the front and rear flippers are aligned with the robot base, the arm is perpendicularly to the latter extended.
Features:
+ Control with a keyboard
+ ROS-based control
+ Output to the real world
+ Output to the simulation
+ Flipper and arm joint rotation constraints
+ Robot sensors information output to ROS
+ Robot state publishing to ROS
+ Initialization of the simulated robot in the initial state
+ Platform state saving and reconfiguration on the start-up to the initial state
***
### Usage
`go build cmd/control_app/control_app.go && ./control_app`
or
`go build cmd/control_app/control_app.go`
`roslaunch control control.launch`
***
### Workflow

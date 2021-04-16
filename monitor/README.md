# Monitor_app
This package provides the monitor functionalities used in the robot_ws
framework to monitor task progress and assign rewards.  
TODO: complete description of functionalities.
***
### Usage 
To launch this node it is necessary to install Go language.  
Manual launch:  
`go run cmd/monitor/monitor_app.go`.   
Build:  
`go build -o bin/monitor_app cmd/monitor/monitor_app.go`.  
The app accepts the port on what ROS_MASTER_URI is located.  
### Subscribes
1. `robot/state` control/State, this srv definition is located within the control package  
2. `safety/relative_deviation` std_msgs/Float32
3. `safety/angular` std_msgs/Float32
4. `odometry` nav_msgs/Odometry
### Publishers
1. `analytics/rollout` std_msgs/String
### Service types
* NewRollout.srv  
  ```
    string  experiment
    int32   seq   
    int32   time_step_limit  
    string  sensors  
    bool    arm  
    bool    angular  
    bool    use_penalty_angular  
    bool    use_penalty_deviation  
    ---
    bool    received
  ```  
* StepReturn.srv  
  ```  
    ---  
    float32 reward  
    bool    done
  ```  
### Services providers
1. `rollout/new` monitor/NewRollout
2. `rollout/start` std_srv/Trigger
3. `rollout/step_return` monitor/StepReturn

### Service clients
1. `goal_info` simulation/GoalInfo
2. `stair_info` simulation/StairInfo

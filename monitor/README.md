# Monitor_app
This package provides the monitor functionalities used in the robot_ws
framework to monitor task progress and assign rewards.  
In the actual framework the Go implementation was replaced by the Python-based one.
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
   This publishes encoded RolloutAnalytics Go struct to the topic   
   which is read by the server where rollout relative data is stored in a database.   
   The next rollout properties of the 
   RolloutState are needed:  
    * ExpSeries -  series of experiments, it relates to the database,  
    * Experiment - current experiment name,  
    * Seq - episode in the current experiment,   
    * Sensors - used sensors,  
    * Arm - indicates whether the is arm used,  
    * Angular - indicates whether the is angular velocity is controlled,  
    * Progress - episode progress,  
    * Reward - episode reward,  
    * AngularM - perceived mean episode angular velocity,  
    * Deviation - perceived mean episode deviation,  
    * Accidents - indicates if something critical happened like tipping over,  
    * TimeSteps - number of time steps spent in the episode.  
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
   Initialization of new rollout.
2. `rollout/start` std_srv/Trigger  
   The node is triggered from the outside since the rollout is started.
3. `rollout/step_return` monitor/StepReturn  
    On every step, we return basic information about the last time step.

### Service clients
1. `goal_info` simulation/GoalInfo
2. `stair_info` simulation/StairInfo

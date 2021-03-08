package monitor

import "C"
import (
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/simulation_structs"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"math"
)

const EXTRADIUS float32 = 0.3

type Core struct {
	rolloutState structs.RolloutState
	ros          ROS
	robotStateCh chan state.State
	odometryCh   chan nav_msgs.Odometry
	initCh       chan bool
	robotState   state.State
	robotPose    nav_msgs.Odometry
	goal         simulation_structs.GoalInfoRes
	timeStep     int32
	stepReqCh    chan bool
	comm 		 map[string]interface{}
}

func (c *Core) Init() {
	c.comm = make(map[string]interface{})
	c.comm["NewRollout"] = c.onNewRollout
	c.comm["StartNewRollout"] = c.onStartNewRollout
	c.comm["StepReturn"] = c.onStepReturn
	c.comm["RobotState"] = make(chan state.State)
	c.comm["Odometry"] = make(chan nav_msgs.Odometry)

	c.rolloutState = structs.RolloutState{}
	c.ros = ROS{}
	c.goal = simulation_structs.GoalInfoRes{}
	c.robotStateCh = make(chan state.State)
	c.robotState = state.State{}
	c.robotPose = nav_msgs.Odometry{}
	c.odometryCh = make(chan nav_msgs.Odometry)
	c.initCh = make(chan bool)
	c.stepReqCh = make(chan bool)

	c.ros.Init(&c.rolloutState, &c.comm) // c.robotStateCh, c.odometryCh, c.initCh, c.stepReqCh
}
func (c *Core) onNewRollout(req *structs.NewRolloutReq, expseries string) {
	c.rolloutState.ExpSeries = expseries
	c.rolloutState.Experiment = req.Experiment
	c.rolloutState.Seq = req.Seq
	c.rolloutState.Sensors = req.Sensors
	c.rolloutState.Arm = req.Arm
	c.rolloutState.Angular = req.Angular
	c.rolloutState.TimeStepLimit = req.TimeStepLimit
	c.rolloutState.Progress = 0.
	c.rolloutState.Reward = 0.
	c.rolloutState.StepReward = 0.
	c.rolloutState.CogDeviation = 0.
	c.rolloutState.StepCogDeviation = 0.
	c.rolloutState.CogHeight = 0.
	c.rolloutState.StepCogHeight = 0.
	c.rolloutState.Done = false
	c.rolloutState.Started = false
	c.rolloutState.Closest = 10000.0
	c.rolloutState.MaximumDist = 0.
	c.rolloutState.Published = false
}
func (c *Core) onStartNewRollout() {
	c.timeStep = 1
	// update goal
	c.ros.goalInfo.Call(&simulation_structs.GoalInfoReq{}, &c.goal)
	// update closest distance
	actualDistToGoal := c.GetDistance()
	c.rolloutState.Closest = actualDistToGoal
	c.rolloutState.MaximumDist = actualDistToGoal
	c.rolloutState.Started = true
}
func (c *Core) onStepReturn() {
	c.timeStep++
	c.rolloutState.StepReward = 0
	c.rolloutState.StepCogDeviation = 0
	c.rolloutState.StepCogHeight = 0
}

func (c *Core) Start() {
	for {
		select {
		case c.robotState = <- c.comm["RobotState"].(chan state.State):
			go c.Estimate()
		case c.robotPose = <- c.comm["Odometry"].(chan nav_msgs.Odometry):
			go c.Estimate()
		}
	}
}
// Be sure to, first, wait untill the robot is spawned at a new location!
// After that you can call create a new rollout e.g. call "new_rollout".

func (c *Core) CheckTippingOver() {

}

func (c *Core) CheckBorders() {

}

func (c *Core) Estimate() {
	dist := c.GetDistance()
	if c.rolloutState.Started && !c.rolloutState.Done {
		if c.rolloutState.Closest - dist > 0.01{
			diff := (c.rolloutState.Closest - dist) / (c.rolloutState.MaximumDist - EXTRADIUS)
			c.rolloutState.Progress += diff
			c.rolloutState.Reward += diff
			c.rolloutState.StepReward += diff
			c.rolloutState.Closest = dist
		}
	}
	c.rolloutState.Done = c.isDone(dist)
	if c.rolloutState.Done && !c.rolloutState.Published {
		c.ros.SendToBackend()
		c.rolloutState.Published = true
		c.goal = simulation_structs.GoalInfoRes{}
	}
}


func (c *Core) isDone(dist float32) bool{
	res := false
	if dist < EXTRADIUS || c.timeStep >= c.rolloutState.TimeStepLimit{
		res = true
	}
	return res
}

func (c *Core) Close() {
	c.ros.Close()
}

func (c *Core) GetDistance() float32 {
	p := []float32{
		float32(c.robotPose.Pose.Pose.Position.X),
		float32(c.robotPose.Pose.Pose.Position.Y),
		float32(c.robotPose.Pose.Pose.Position.Z),
	}
	q := []float32{
		float32(c.goal.X),
		float32(c.goal.Y),
		float32(c.goal.Z),
	}
	dist := math.Pow(float64(p[0]-q[0]), 2)
	dist += math.Pow(float64(p[1]-q[1]), 2)
	dist += math.Pow(float64(p[2]-q[2]), 2)
	return float32(math.Pow(dist, 0.5))
}



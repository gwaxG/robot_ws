package monitor

import (
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/simulation_structs"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"log"
	"math"
)



type Core struct {
	rolloutState 			structs.RolloutState
	ros 			ROS
	robotStateCh	chan state.State
	odometryCh		chan nav_msgs.Odometry
	initCh			chan bool
	robotState		state.State
	robotPose		nav_msgs.Odometry
	goal			*simulation_structs.GoalInfoRes
}

func (c *Core) Init() {
	c.rolloutState = structs.RolloutState{}
	c.ros = ROS{}
	c.goal = nil
	c.robotStateCh = make(chan state.State)
	c.robotState = state.State{}
	c.robotPose = nav_msgs.Odometry{}
	c.odometryCh = make(chan nav_msgs.Odometry)
	c.initCh = make(chan bool)

	c.ros.Init(&c.rolloutState, c.robotStateCh, c.odometryCh, c.initCh)
}

func (c *Core) Start () {
	for {
		select {
		case _ = <- c.initCh:
			go c.PreRolloutInit()
		case c.robotState = <- c.robotStateCh:
			go c.Estimate()
		case c.robotPose = <- c.odometryCh:
			go c.Estimate()
		}
	}
}
// Be sure to, first, wait untill the robot is spawned at a new location!
// After that you can call create a new rollout e.g. call "new_rollout".
func (c *Core) PreRolloutInit() {
	// update goal
	c.ros.goalInfo.Call(&simulation_structs.GoalInfoReq{}, c.goal)
	// update closest distance
	c.rolloutState.Closest = c.GetDistance()
	c.rolloutState.MaximumDist = c.GetDistance()
}

func (c *Core) Estimate() {
	log.Println("State ", c.robotState)
	log.Println("State ", c.robotPose)
	if c.rolloutState.Started && !c.rolloutState.Done {
		log.Println("It was started")
		dist := c.GetDistance()
		if dist < c.rolloutState.Closest {
			diff := (c.rolloutState.Closest - dist) / c.rolloutState.MaximumDist
			c.rolloutState.Progress += diff
			c.rolloutState.Reward += diff
			c.rolloutState.StepReward += diff
		}
	}
	if c.rolloutState.Done && !c.rolloutState.Published {
		c.ros.SendToBackend()
		c.rolloutState.Published = true
		c.goal = nil
	}
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
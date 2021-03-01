package monitor

import (
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"log"
	"math"
)



type Core struct {
	rolloutState 			structs.RolloutState
	ros 			ROS
	robotStateCh	chan state.State
	odometryCh		chan nav_msgs.Odometry
	robotState		state.State
	robotPose		nav_msgs.Odometry
}

func (c *Core) Init() {
	c.rolloutState = structs.RolloutState{}
	c.ros = ROS{}
	c.ros.Init(&c.rolloutState, c.robotStateCh, c.odometryCh)
	c.robotStateCh = make(chan state.State)
	c.robotState = state.State{}
	c.robotPose = nav_msgs.Odometry{}
	c.odometryCh = make(chan nav_msgs.Odometry)
}

func (c *Core) Start () {
	for {
		select {
		case c.robotState = <- c.robotStateCh:
			go c.Estimate()
		case c.robotPose = <- c.odometryCh:
			go c.Estimate()
		}
	}
}

func (c *Core) Estimate() {
	log.Println("State ", c.robotState)
	log.Println("State ", c.robotPose)
	if c.rolloutState.Started && !c.rolloutState.Done {
		log.Println("It was started")
	}
	if c.rolloutState.Done && !c.rolloutState.Published {
		c.ros.SendToBackend()
		c.rolloutState.Published = true
	}
}

func (c *Core) Close() {
	c.ros.Close()
}



func GetDistance(p, q []float32) float32 {
	dist := math.Pow(float64(p[0]-q[0]), 2)
	dist += math.Pow(float64(p[1]-q[1]), 2)
	dist += math.Pow(float64(p[2]-q[2]), 2)
	return float32(math.Pow(dist, 0.5))
}
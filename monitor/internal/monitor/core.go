package monitor

import "C"
import (
	"log"
	"math"
	"sync"

	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/simulation_structs"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
)

const EXTRADIUS float32 = 0.3
const TippingReward float32 = -1.0

var mutexEstimate = sync.Mutex{}

type Core struct {
	rolloutState     structs.RolloutState
	ros              ROS
	robotStateCh     chan state.State
	odometryCh       chan nav_msgs.Odometry
	initCh           chan bool
	robotState       state.State
	robotPose        nav_msgs.Odometry
	goal             simulation_structs.GoalInfoRes
	timeStep         int32
	stepReqCh        chan bool
	comm             map[string]interface{}
	flushCache       float32
	cumulatedPenalty []float32
	cumulation       [][]float32
	normalized       bool
	normalization    float32
	debug            float32
}

func (c *Core) Init() {
	c.comm = make(map[string]interface{})
	c.comm["NewRollout"] = c.onNewRollout
	c.comm["StartNewRollout"] = c.onStartNewRollout
	c.comm["StepReturn"] = c.onStepReturn
	c.comm["RobotState"] = make(chan state.State)
	c.comm["Odometry"] = make(chan nav_msgs.Odometry)
	c.comm["SafetyDeviation"] = make(chan float32)
	c.comm["SafetyAngular"] = make(chan float32)

	c.rolloutState = structs.RolloutState{}
	c.ros = ROS{}
	c.goal = simulation_structs.GoalInfoRes{}
	c.robotStateCh = make(chan state.State)
	c.robotState = state.State{}
	c.robotPose = nav_msgs.Odometry{}
	c.odometryCh = make(chan nav_msgs.Odometry)
	c.initCh = make(chan bool)
	c.stepReqCh = make(chan bool)
	c.cumulatedPenalty = []float32{}
	c.cumulation = [][]float32{[]float32{}, []float32{}}
	c.normalized = false
	c.normalization = 0.

	c.debug = 0.

	c.ros.Init(&c.rolloutState, &c.comm) // c.robotStateCh, c.odometryCh, c.initCh, c.stepReqCh
}

func (c *Core) addToCumulation() {
	// Mean deviation during the staircase traversal.
	cumulation := sumFloat32(&c.cumulatedPenalty)
	cumulationMean := meanFloat32(&c.cumulatedPenalty)
	// Number of time steps during the traversal.
	cumulationLength := len(c.cumulatedPenalty)
	c.cumulatedPenalty = []float32{}
	// log.Println("adding", cumulation)
	if cumulation < 1.0 {
		return
	}
	L := 10
	if len(c.cumulation[0]) < L {
		// c.cumulation = append(c.cumulation, cumulation)
		c.cumulation[0] = append(c.cumulation[0], float32(cumulation))
		c.cumulation[1] = append(c.cumulation[1], float32(cumulationLength))
	}
	if len(c.cumulation[0]) == L {
		c.normalized = true
		// Definition of normalization factor.
		// c.normalization = 1.0 / (meanFloat32(&c.cumulation[0]) * meanFloat32(&c.cumulation[1]))
		// c.normalization = 1.0 / (meanFloat32(&c.cumulation[0]))
		c.normalization = 1.0 / (maxFloat32(&c.cumulation[0]))
	}
	_ = cumulationMean
}

func (c *Core) getNormalization() float32 {
	if c.normalized {
		// log.Println("Normalized", c.normalization)
		// log.Println("norm", c.normalization)
		return c.normalization
	} else {
		// log.Println("not normalized", len(c.cumulation))
		return 0.
	}
}

func (c *Core) onNewRollout(req *structs.NewRolloutReq, expseries string) {
	// Trigger normalization since the experiment has been changed.
	if c.rolloutState.Experiment != req.Experiment {
		if req.UsePenaltyAngular || req.UsePenaltyDeviation {
			c.normalized = false
			c.normalization = 0.
			c.cumulatedPenalty = []float32{}
			c.cumulation = [][]float32{[]float32{}, []float32{}}
		} else {
			c.normalized = true
		}
	}
	c.debug = 0.
	c.rolloutState.ExpSeries = expseries
	c.rolloutState.Experiment = req.Experiment
	c.rolloutState.Seq = req.Seq
	c.rolloutState.Sensors = req.Sensors
	c.rolloutState.Arm = req.Arm
	c.rolloutState.Angular = req.Angular
	c.rolloutState.TimeStepLimit = req.TimeStepLimit
	c.rolloutState.UsePenaltyDeviation = req.UsePenaltyDeviation
	c.rolloutState.UsePenaltyAngular = req.UsePenaltyAngular
	c.rolloutState.Progress = 0.
	c.rolloutState.Reward = 0.
	c.rolloutState.StepReward = 0.
	c.rolloutState.StepDeviation = []float32{}
	c.rolloutState.StepAngular = []float32{}
	c.rolloutState.AngularM = []float32{}
	c.rolloutState.Deviation = []float32{}
	c.rolloutState.Done = false
	c.rolloutState.TippingOverReward = 0
	c.rolloutState.Started = false
	c.rolloutState.Closest = 10000.0
	c.rolloutState.MaximumDist = 0.
	c.rolloutState.Published = false
	c.rolloutState.EverStarted = false
	c.rolloutState.TimeSteps = 0
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
	c.rolloutState.EverStarted = false
	c.rolloutState.Done = false
	c.flushCache = 0
}
func (c *Core) onStepReturn() float32 {
	c.timeStep++
	c.rolloutState.TimeSteps = int(c.timeStep)
	if c.rolloutState.Done {
		return c.flushCache
	}
	return c.flushStepResults()
}

func (c *Core) Start() {
	for {
		select {
		case c.robotState = <-c.comm["RobotState"].(chan state.State):
			go func() {
				c.Estimate()
			}()
		case c.robotPose = <-c.comm["Odometry"].(chan nav_msgs.Odometry):
			go func() {
				c.CheckTippingOver()
				c.Estimate()
			}()
		case deviation := <-c.comm["SafetyDeviation"].(chan float32):
			c.rolloutState.StepDeviation = append(c.rolloutState.StepDeviation, deviation)
		case angular := <-c.comm["SafetyAngular"].(chan float32):
			c.rolloutState.StepAngular = append(c.rolloutState.StepAngular, angular)
		}
	}
}

// Be sure to, first, wait untill the robot is spawned at a new location!
// After that you can call create a new rollout e.g. call "new_rollout".
func (c *Core) CheckTippingOver() {
	roll, pitch, _ := c.getEuler()
	accident := false
	if roll > math.Pi/2 && !c.rolloutState.Done {
		c.rolloutState.Accidents = "Front tipping over"
		accident = true
	} else if roll < -math.Pi/2 && !c.rolloutState.Done {
		c.rolloutState.Accidents = "Rear tipping over"
		accident = true
	}
	if pitch > math.Pi/2 && !c.rolloutState.Done {
		c.rolloutState.Accidents = "Right tipping over"
	} else if pitch < -math.Pi/2 && !c.rolloutState.Done {
		c.rolloutState.Accidents = "Left tippîng over"
		accident = true
	}
	if accident {
		c.rolloutState.Done = true
		c.rolloutState.TippingOverReward += TippingReward
	}
}

func (c *Core) CheckBorders() {
	panic("CheckBorders is not implemented.")
}

func (c *Core) flushStepResults() float32 {
	var stepReward float32
	meanAngular := meanFloat32(&c.rolloutState.StepAngular)
	if c.rolloutState.UsePenaltyAngular {
		if !c.normalized {
			c.cumulatedPenalty = append(c.cumulatedPenalty, meanAngular)
		}
		stepReward -= meanAngular * c.getNormalization()
	}
	c.rolloutState.AngularM = append(c.rolloutState.AngularM, meanAngular)
	c.rolloutState.StepAngular = []float32{}

	meanDeviation := meanFloat32(&c.rolloutState.StepDeviation)
	if c.rolloutState.UsePenaltyDeviation {
		if !c.normalized {
			c.cumulatedPenalty = append(c.cumulatedPenalty, meanDeviation)
		}
		c.debug += meanDeviation * c.getNormalization()
		stepReward -= meanDeviation * c.getNormalization()
	}
	// log.Println(" step correction", meanDeviation*c.getNormalization())
	c.rolloutState.Deviation = append(c.rolloutState.Deviation, meanDeviation)
	c.rolloutState.StepDeviation = []float32{}

	// log.Println(" step reward", c.rolloutState.StepReward)
	stepReward += c.rolloutState.StepReward
	c.rolloutState.StepReward = 0

	stepReward += c.rolloutState.TippingOverReward
	c.rolloutState.TippingOverReward = 0

	c.rolloutState.Reward += stepReward

	return stepReward
}

func (c *Core) Estimate() {
	mutexEstimate.Lock()
	defer mutexEstimate.Unlock()
	dist := c.GetDistance()
	if c.rolloutState.Started && !c.rolloutState.Done {
		if c.rolloutState.Closest-dist > 0.01 {
			diff := (c.rolloutState.Closest - dist) / (c.rolloutState.MaximumDist - EXTRADIUS)
			// To fight against great drop of progress, we add a check.
			if diff < 0. {
				diff = 0.
				log.Printf("Inconsistent distances where diff is %f\n", diff)
			}
			c.rolloutState.Progress += diff
			// c.rolloutState.Reward += diff
			c.rolloutState.StepReward += diff
			c.rolloutState.Closest = dist
		}
	}
	if !c.rolloutState.Done {
		c.rolloutState.Done = c.isDoneByDistance(dist)
	}
	// fmt.Println("Need to send?", c.rolloutState.Done, !c.rolloutState.Published, c.rolloutState.Started)
	if c.rolloutState.Done && !c.rolloutState.Published && c.rolloutState.Started {
		// flush results
		c.flushCache = c.flushStepResults()
		if !c.normalized {
			c.addToCumulation()
		}
		// log.Println("debug ", c.debug, c.normalized, c.normalization, len(c.cumulation[0]))
		c.ros.SendToBackend()
		c.rolloutState.Published = true
		c.goal = simulation_structs.GoalInfoRes{}
	}
}

func (c *Core) isDoneByDistance(dist float32) bool {
	res := false
	if dist < EXTRADIUS || c.timeStep >= c.rolloutState.TimeStepLimit {
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

func (c *Core) getEuler() (float64, float64, float64) {
	X := c.robotPose.Pose.Pose.Orientation.X
	Y := c.robotPose.Pose.Pose.Orientation.Y
	Z := c.robotPose.Pose.Pose.Orientation.Z
	W := c.robotPose.Pose.Pose.Orientation.W
	k := math.Sqrt(W*W + X*X + Y*Y + Z*Z)
	X = X / k
	Y = Y / k
	Z = Z / k
	W = W / k
	roll := math.Atan2(2*(W*X+Y*Z), 1-2*(X*X+Y*Y))
	pitch := math.Asin(2 * (W*Y - Z*X))
	yaw := math.Atan2(2*(X*Y+W*Z), 1-2*(Y*Y+Z*Z))
	return roll, pitch, yaw
}

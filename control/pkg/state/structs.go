package state

import "math"

type State struct {
	FrontFlippers float64
	RearFlippers  float64
	Linear        float64
	Angular       float64
	ArmJoint1     float64
	ArmJoint2     float64
	ArmJoint3     float64 // not supported
	ArmJoint4     float64 // not supported
}

func (cs *State) Reset () {
	cs.FrontFlippers = 0.
	cs.RearFlippers = 0.
	cs.Linear = 0.
	cs.Angular = 0.
	cs.ArmJoint1 = 0.
	cs.ArmJoint2 = 0.
	cs.ArmJoint3 = 0.
	cs.ArmJoint4 = 0.
}

type Limit struct {
	Min, Max float64
}

type Limits struct {
	FrontFlippers Limit
	RearFlippers  Limit
	Linear 		  Limit
	Angular       Limit
	ArmJoint1     Limit
	ArmJoint2     Limit
	ArmJoint3     Limit
	ArmJoint4     Limit
}

func (sl *Limits) Init() {
	sl.Angular = Limit{-1.0, 1.0}
	sl.Linear = Limit{-1.0, 1.0}
	sl.FrontFlippers = Limit{-math.Pi/4,math.Pi/4}
	sl.RearFlippers = Limit{-math.Pi/4,math.Pi/4}
	sl.ArmJoint1 = Limit{-math.Pi/4,math.Pi/4}
	sl.ArmJoint2 = Limit{-math.Pi/4,math.Pi/4}
}
package state

type State struct {
	FrontFlippers float64
	RearFlippers float64
	Linear float64
	Angular float64
	ArmJoint1 float64
	ArmJoint2 float64
	ArmJoint3 float64 // not supported
	ArmJoint4 float64 // not supported
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


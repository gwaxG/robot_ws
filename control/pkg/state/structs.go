package state

import (
	"github.com/aler9/goroslib/pkg/msg"
	"math"
)

type State struct {
	msg.Package `ros:"control"`
	Linear        float64
	Angular       float64
	FrontFlippers float64
	RearFlippers  float64
	ArmJoint1     float64
	ArmJoint2     float64
	ArmJoint3     float64 // not supported
	ArmJoint4     float64 // not supported
}

func Reset (cs *State) {
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

type Sensor struct {
	msg.Package `ros:"control"`
	FrontFlipperCurrent   float64 `json:"frontFlipperCurrent"`
	RearFlipperCurrent    float64 `json:"rearFlipperCurrent"`
	Voltage               float64 `json:"voltage"`
	AccelX                float64 `json:"accelX"`
	AccelY                float64 `json:"accelY"`
	AccelZ                float64 `json:"accelZ"`
	GyroX                 float64 `json:"gyroX"`
	GyroY                 float64 `json:"gyroY"`
	GyroZ                 float64 `json:"gyroZ"`
	Yaw                   float64 `json:"yaw"`
	LeftMotorCountsFront  int64   `json:"leftMotorCountsFront"`
	LeftMotorCountsRear   int64   `json:"leftMotorCountsRear"`
	RightMotorCountsFront int64   `json:"rightMotorCountsFront"`
	RightMotorCountsRear  int64   `json:"rightMotorCountsRear"`
	FrontFlipperCounts    int64   `json:"frontFlipperCounts"`
	RearFlipperCounts     int64   `json:"rearFlipperCounts"`
}

type saveState struct {
	FrontFlippers   float64 `json:"FrontFlippers"`
	RearFlippers	float64 `json:"RearFlippers"`
	ArmJoint1 		float64 `json:"ArmJoint1"`
	ArmJoint2 		float64 `json:"ArmJoint2"`
	ArmJoint3 		float64 `json:"ArmJoint3"`
	ArmJoint4 		float64 `json:"ArmJoint4"`
}

package structs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

//-----------------------------
type RolloutState struct {
	ExpSeries           string
	Experiment          string
	Seq                 int32
	Sensors             string
	Arm                 bool
	Angular             bool
	TimeStepLimit       int32
	TippingOverReward   float32
	Progress            float32
	Reward              float32
	StepReward          float32
	UsePenaltyDeviation bool
	UsePenaltyAngular   bool
	Deviation           []float32
	AngularM            []float32
	StepDeviation       []float32
	StepAngular         []float32
	Done                bool
	Started             bool
	Closest             float32
	MaximumDist         float32
	Published           bool
	Accidents           string
	EverStarted         bool
	TimeSteps           int
}

//-----------------------------
type RolloutAnalytics struct {
	ExpSeries  string  `bson:"exp_series" json:"exp_series"`
	Experiment string  `bson:"experiment" json:"experiment"`
	Seq        int32   `bson:"seq" json:"seq"`
	Sensors    string  `bson:"Sensors" json:"Sensors"`
	Arm        bool    `bson:"arm" json:"arm"`
	Angular    bool    `bson:"angular" json:"angular"`
	Progress   float32 `bson:"progress" json:"progress"`
	Reward     float32 `bson:"reward" json:"reward"`
	Deviation  float32 `bson:"deviation" json:"deviation"`
	AngularM   float32 `bson:"angularm" json:"angularm"`
	Accidents  string  `bson:"accidents" json:"accidents"`
	TimeSteps  int     `bson:"time_steps" json:"time_steps"`
}

//-----------------------------
type NewRolloutReq struct {
	Experiment          string
	Seq                 int32
	TimeStepLimit       int32
	Sensors             string
	Arm                 bool
	Angular             bool
	UsePenaltyAngular   bool
	UsePenaltyDeviation bool
}

type NewRolloutRes struct {
	Received bool
}

type NewRollout struct {
	msg.Package `ros:"monitor"`
	NewRolloutReq
	NewRolloutRes
}

//-----------------------------
type StepReturnReq struct{}

type StepReturnRes struct {
	Reward float32
	Done   bool
}

type StepReturn struct {
	msg.Package `ros:"monitor"`
	StepReturnReq
	StepReturnRes
}

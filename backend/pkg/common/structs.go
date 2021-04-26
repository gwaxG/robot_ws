package common

import "github.com/aler9/goroslib/pkg/msg"

//-----------------------------
type RolloutAnalytics struct {
	msg.Package `ros:"monitor"`
	ExpSeries   string  `bson:"exp_series" json:"exp_series"`
	Experiment  string  `bson:"experiment" json:"experiment"`
	Seq         int32   `bson:"seq" json:"seq"`
	Sensors     string  `bson:"Sensors" json:"Sensors"`
	Arm         bool    `bson:"arm" json:"arm"`
	Angular     bool    `bson:"angular" json:"angular"`
	Progress    float32 `bson:"progress" json:"progress"`
	Reward      float32 `bson:"reward" json:"reward"`
	Deviation   float32 `bson:"deviation" json:"deviation"`
	AngularM    float32 `bson:"angularm" json:"angularm"`
	Accidents   string  `bson:"accidents" json:"accidents"`
	TimeSteps   int     `bson:"time_steps" json:"time_steps"`
}

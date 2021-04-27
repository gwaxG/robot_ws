package common

import "github.com/aler9/goroslib/pkg/msg"

//-----------------------------
type RolloutAnalytics struct {
	msg.Package `ros:"monitor"`
	ExpSeries   string  `bson:"exp_series" json:"exp_series"`
	Experiment  string  `bson:"experiment" json:"experiment"`
	Seq         int32   `bson:"seq" json:"seq"`
	Sensors     string  `bson:"sensors" json:"sensors"`
	Arm         bool    `bson:"arm" json:"arm"`
	Angular     bool    `bson:"angular" json:"angular"`
	Progress    float32 `bson:"progress" json:"progress"`
	Reward      float32 `bson:"reward" json:"reward"`
	AngularM    float32 `bson:"angular_m" json:"angular_m"`
	Deviation   float32 `bson:"deviation" json:"deviation"`
	Accidents   string  `bson:"accidents" json:"accidents"`
	TimeSteps   int32   `bson:"time_steps" json:"time_steps"`
	Log         string  `bson:"log" json:"log"`
	Debug       float32 `bson:"debug" json:"debug"`
}

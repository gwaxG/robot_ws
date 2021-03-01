package structs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

//-----------------------------
type RolloutState struct {
	Experiment  		string
	Seq 				int32
	Sensors 			string
	Arm					bool
	Angular				bool
	Progress			float32
	Reward				float32
	StepReward			float32
	CogDeviation 		float32
	StepCogDeviation 	float32
	CogHeight			float32
	StepCogHeight		float32
	Done 				bool
	Started 			bool
	Closest				float32
	Published 			bool
}

//-----------------------------
type AnalyticsReq struct {
	Experiment  	string
	Seq 			int32
	Sensors 		string
	Arm				bool
	Angular			bool
	Progress		float32
	Reward			float32
	CogDeviation 	float32
	CogHeight		float32
}

type AnalyticsRes struct {
	Received		bool
}

type Analytics struct {
	msg.Package `ros:"monitor"`
	AnalyticsReq
	AnalyticsRes
}

//-----------------------------
type NewRolloutReq struct {
	Experiment  	string
	Sensors 		string
	Seq 			int32
	Arm				bool
	Angular			bool
}

type NewRolloutRes struct {
	Received		bool
}

type NewRollout struct {
	msg.Package `ros:"monitor"`
	AnalyticsReq
	AnalyticsRes
}

//-----------------------------
type StepReturnReq struct {}

type StepReturnRes struct {
	Reward			float32
	Done			bool
}

type StepReturn struct {
	msg.Package `ros:"monitor"`
	StepReturnReq
	StepReturnRes
}


package simulation_structs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GoalInfoReq struct{}

type GoalInfoRes struct {
	X    float64
	Y    float64
	Z    float64
	Task string
	Rand bool
}

type GoalInfo struct {
	msg.Package `ros:"monitor"`
	GoalInfoReq
	GoalInfoRes
}

type StairInfoReq struct{}

type StairInfoRes struct {
	Length float64
	Height float64
	Number float64
	Exist  bool
}

type StairInfo struct {
	msg.Package `ros:"monitor"`
	StairInfoReq
	StairInfoRes
}

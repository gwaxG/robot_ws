package input

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

type Ros struct {
	stateChange chan state.State
	state state.State
}

type JaguarControl struct {
	// JaguarControl.msg Go definition
	msg.Package `ros:"control"`
	Linear  float64
	Angular float64
	FrontFlippers float64
	RearFlippers float64
	ArmJoint1  float64
	ArmJoint2  float64
	ArmJoint3  float64
	ArmJoint4  float64
}

func (r *Ros) Init (stateChange chan state.State) {
	r.stateChange = stateChange
	r.state = state.State{}
}

func (r * Ros) Serve () {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "jag_control",
		MasterAddress: "127.0.0.1:11311",
	})
	utils.FailOnError(err, "Failed to connect to ROS master")
	defer func(){err=n.Close(); utils.FailOnError(err, "Failed to disconnect from ROS master")}()

	subBase, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "platform_cmd",
		Callback: r.onRequest,
	})
	utils.FailOnError(err, "Failed to create platform_cmd subscriber")
	defer func(){err=subBase.Close(); utils.FailOnError(err, "Failed to disconnect from platform_cmd")}()

	select {}
}

func (r * Ros) onRequest (msg *JaguarControl) {
	r.state.Linear = msg.Linear
	r.state.Angular = msg.Angular
	r.state.FrontFlippers = msg.FrontFlippers
	r.state.RearFlippers = msg.RearFlippers
	r.state.ArmJoint1 = msg.ArmJoint1
	r.state.ArmJoint2 = msg.ArmJoint2
	r.state.ArmJoint3 = msg.ArmJoint3
	r.state.ArmJoint4 = msg.ArmJoint4
	r.stateChange <- r.state
}

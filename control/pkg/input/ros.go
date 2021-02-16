package input

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

type Ros struct {
	stateChange chan state.State
	state state.State
}

func (r *Ros) Init (stateChange chan state.State) {
	r.stateChange = stateChange
	r.state = state.State{}
}

func (r * Ros) Serve () {
	// create a node and connect to the master
	subBase, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     connections.RosConnection(),
		Topic:    "robot_cmd",
		Callback: r.onRequest,
	})
	utils.FailOnError(err, "Failed to create platform_cmd subscriber")
	defer func(){err=subBase.Close(); utils.FailOnError(err, "Failed to disconnect from platform_cmd")}()

	select {}
}

func (r * Ros) onRequest (msg *state.State) {
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

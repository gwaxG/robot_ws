package input

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

type Ros struct {
	stateChange chan state.State
	state 		state.State
	reset 		chan bool
}

func (r *Ros) Init (stateChange chan state.State, reset chan bool) {
	r.reset = reset
	r.stateChange = stateChange
	r.state = state.State{}
}

func (r * Ros) Serve () {
	var err error
	var subBase *goroslib.Subscriber
	var servReset *goroslib.ServiceProvider

	subBase, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     connections.RosConnection(),
		Topic:    "robot_cmd",
		Callback: r.onRequest,
	})
	utils.FailOnError(err, "Failed to create platform_cmd subscriber")
	defer func(){err=subBase.Close(); utils.FailOnError(err, "Failed to disconnect from robot_cmd")}()

	servReset, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     connections.RosConnection(),
		Name:     "robot/reset",
		Srv:      &std_srvs.Trigger{},
		Callback: r.onResetService,
	})
	utils.FailOnError(err, "Failed to create platform_cmd subscriber")
	defer func(){err=servReset.Close(); utils.FailOnError(err, "Failed to disconnect from reset service")}()

	select {}
}

func (r * Ros) onResetService(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	r.reset <- true
	return &std_srvs.TriggerRes{
		Success: true,
		Message: "Request handled",
	}
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

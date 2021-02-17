package output

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
)

type RosCmd struct {
	pubArm1      	*goroslib.Publisher
	pubArm2      	*goroslib.Publisher
	pubBase      	*goroslib.Publisher
	pubFlipper   	*goroslib.Publisher
	msgFloat 		*std_msgs.Float64
	seqBase 		uint32
	seqFlipper 		uint32
	stateActionChan chan []state.State
}

func (p *RosCmd) Init(stateActionChan chan []state.State) {
	// chan init
	p.stateActionChan = stateActionChan
	p.seqBase = 1
	p.seqFlipper = 1
	// /cmd_vel TwistStamped
	p.pubBase, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  connections.RosConnection(),
		Topic: "cmd_vel",
		Msg:   &geometry_msgs.TwistStamped{},
	})
	// /cmd_flipper TwistStamped
	p.pubFlipper, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  connections.RosConnection(),
		Topic: "cmd_flipper",
		Msg:   &geometry_msgs.TwistStamped{},
	})

	// /jaguar/arm_1_effort_controller/command Float64
	p.pubArm1, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  connections.RosConnection(),
		Topic: "jaguar/arm_1_effort_controller/command",
		Msg:   &std_msgs.Float64{},
	})

	// /jaguar/arm_2_effort_controller/command Float64
	p.pubArm2, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  connections.RosConnection(),
		Topic: "jaguar/arm_2_effort_controller/command",
		Msg:   &std_msgs.Float64{},
	})

	// Initialize to the extended state
	var initState state.State
	p.ServeBase(&initState)
	p.ServeFlippers(&initState)
	p.ServeArm(&initState)
}

func (p *RosCmd) ServeArm(actions *state.State) {
	// if actions.ArmJoint1 != 0.0 || actions.ArmJoint2 != 0.0 {}
	p.pubArm1.Write(&std_msgs.Float64{Data: actions.ArmJoint1})
	p.pubArm2.Write(&std_msgs.Float64{Data: actions.ArmJoint2})
}

func (p *RosCmd) ServeBase(actions *state.State) {
	msgStampedTwist := &geometry_msgs.TwistStamped{
		Header: std_msgs.Header{
			Seq: p.seqBase,
			Stamp: connections.RosConnection().TimeNow(),
		},
		Twist: geometry_msgs.Twist{
			Linear:  geometry_msgs.Vector3{
				X: actions.Linear,
			},
			Angular: geometry_msgs.Vector3{
				Z: actions.Angular,
			},
		},
	}
	p.pubBase.Write(msgStampedTwist)
	p.seqBase += 1
}

func (p *RosCmd) ServeFlippers(actions *state.State) {
	// if actions.FrontFlippers != 0.0 || actions.RearFlippers != 0.0 {}
	msgStampedTwist := &geometry_msgs.TwistStamped{
		Header: std_msgs.Header{
			Seq: p.seqFlipper,
			Stamp: connections.RosConnection().TimeNow(),
		},
		Twist: geometry_msgs.Twist{
			Linear:  geometry_msgs.Vector3{
				X: actions.FrontFlippers,
				Y: actions.FrontFlippers,
			},
			Angular: geometry_msgs.Vector3{
				X: actions.RearFlippers,
				Y: actions.RearFlippers,
			},
		},
	}
	p.pubFlipper.Write(msgStampedTwist)
	p.seqFlipper += 1
}

func (p *RosCmd) Serve() {
	var StateChange []state.State
	var StateAction state.State

	for {
		StateChange = <- p.stateActionChan
		StateAction = StateChange[0]
		p.ServeBase(&StateAction)
		p.ServeFlippers(&StateAction)
		p.ServeArm(&StateAction)
	}
}

func (p *RosCmd) Close() {
	_ = p.pubBase.Close()
	_ = p.pubArm1.Close()
	_ = p.pubArm2.Close()
	_ = p.pubFlipper.Close()
}


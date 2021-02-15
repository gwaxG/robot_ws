package output

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"log"
	"time"
)

type RosCmd struct {
	node 			*goroslib.Node
	pubArm1      	*goroslib.Publisher
	pubArm2      	*goroslib.Publisher
	pubBase      	*goroslib.Publisher
	pubFlipper   	*goroslib.Publisher
	msgStampedTwist *geometry_msgs.TwistStamped
	msgFloat 		*std_msgs.Float64
	seqBase 		uint32
	seqFlipper 		uint32
	actions 		chan state.State
}

func (p *RosCmd) Init(node *goroslib.Node, actions chan state.State) {
	//
	p.node = node
	// chan init
	p.actions = actions
	// /cmd_vel TwistStamped

	p.pubBase, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  p.node,
		Topic: "cmd_vel",
		Msg:   &geometry_msgs.TwistStamped{},
	})

	// /cmd_flipper TwistStamped
	p.pubFlipper, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  p.node,
		Topic: "cmd_flipper",
		Msg:   &geometry_msgs.TwistStamped{},
	})

	// /jaguar/arm_1_effort_controller/command Float64
	p.pubArm1, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  p.node,
		Topic: "jaguar/arm_1_effort_controller/command",
		Msg:   &std_msgs.Float64{},
	})
	// /jaguar/arm_2_effort_controller/command Float64
	p.pubArm2, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  p.node,
		Topic: "jaguar/arm_2_effort_controller/command",
		Msg:   &std_msgs.Float64{},
	})

	// Float64 msg
	p.msgFloat = &std_msgs.Float64{
		Data: 0.0,
	}

	// stamped twist msg
	p.msgStampedTwist = &geometry_msgs.TwistStamped{
		Header: std_msgs.Header{Stamp:   time.Time{}.UTC()},
		Twist: geometry_msgs.Twist{Linear:  geometry_msgs.Vector3{}, Angular: geometry_msgs.Vector3{}},
	}

}

func (p *RosCmd) ServeArm(actions *state.State) {
	if actions.ArmJoint1 != 0.0 || actions.ArmJoint2 != 0.0 {
		p.msgFloat.Data = actions.ArmJoint1
		p.pubArm1.Write(p.msgFloat)
		p.msgFloat.Data = actions.ArmJoint2
		p.pubArm2.Write(p.msgFloat)
	}
}

func (p *RosCmd) ServeBase(actions *state.State) {
	log.Println("Msg in ServeBase", actions)
	p.msgStampedTwist.Header.Seq = p.seqBase
	p.msgStampedTwist.Header.Stamp = p.node.TimeNow()
	p.msgStampedTwist.Twist.Linear.X = actions.Linear
	p.msgStampedTwist.Twist.Angular.Z = actions.Angular
	p.pubBase.Write(p.msgStampedTwist)
	p.seqBase += 1
}

func (p *RosCmd) ServeFlippers(actions *state.State) {
	/*
		self.msg_flipper.twist.linear.x = self.flipper_front
		self.msg_flipper.twist.linear.y = self.flipper_front
		self.msg_flipper.twist.angular.x = self.flipper_rear
		self.msg_flipper.twist.angular.y = self.flipper_rear
	*/
	if actions.FrontFlippers != 0.0 || actions.RearFlippers != 0.0 {
		p.msgStampedTwist.Header.Seq = p.seqFlipper
		p.msgStampedTwist.Header.Stamp = p.node.TimeNow()
		// Check git
		p.msgStampedTwist.Twist.Linear.X = actions.FrontFlippers
		p.msgStampedTwist.Twist.Linear.Y = actions.FrontFlippers
		p.msgStampedTwist.Twist.Angular.X = actions.RearFlippers
		p.msgStampedTwist.Twist.Angular.Y = actions.RearFlippers
		p.pubFlipper.Write(p.msgStampedTwist)
		p.seqFlipper += 1
	}
}

func (p *RosCmd) Serve() {
	actions := state.State{}
	for {
		actions = <- p.actions
		log.Println("Action received")
		p.ServeBase(&actions)
		p.ServeFlippers(&actions)
		p.ServeArm(&actions)
	}
}

func (p *RosCmd) Close() {
	_ = p.pubBase.Close()
	_ = p.pubArm1.Close()
	_ = p.pubArm2.Close()
	_ = p.pubFlipper.Close()
}

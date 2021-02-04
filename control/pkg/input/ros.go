package input

import (
	"fmt"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"log"
)

type Ros struct {}

func (r * Ros) Serve (stateChange chan state.State) {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "jag_control",
		MasterAddress: "127.0.0.1:11311",
	})
	log.Println("")
	utils.FailOnError(err, "Failed to connect to ROS master")
	defer utils.FailOnError(n.Close(), "Failed to close jag_control node")

	subBase, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "base_cmd",
		Callback: r.onBase,
	})
	utils.FailOnError(err, "Failed to create base_cmd subscriber")

	subArm, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "arm_cmd",
		Callback: r.onArm,
	})
	utils.FailOnError(err, "Failed to create arm_cmd subscriber")
	defer utils.FailOnError(subArm.Close(), "Failed to close sub_arm")
	defer utils.FailOnError(subBase.Close(), "Failed to close sub_base")
	select {}
}

func (r * Ros) onArm (msg *geometry_msgs.Twist) {
	fmt.Printf("Incoming: %+v\n", msg)
}


func (r * Ros) onBase (msg *geometry_msgs.Twist) {
	fmt.Printf("Incoming: %+v\n", msg)
}

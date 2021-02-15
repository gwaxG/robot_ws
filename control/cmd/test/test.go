package main

import (
"fmt"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
"github.com/gwaxG/robot_ws/control/pkg/connections"
"github.com/aler9/goroslib"
"github.com/aler9/goroslib/pkg/msg"
)

// define a custom message.
// unlike the standard library, a .msg file is not needed.
// a structure definition is enough.
type TestMessage struct {
	msg.Package `ros:"my_package"`
	FirstField  uint32
	SecondField string
}

func main() {
	// create a node and connect to the master
	n := connections.ConnectRos(false)
	defer n.Close()

	// create a publisher
	pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &std_msgs.Bool{},
	})
	if err != nil {
		panic(err)
	}
	defer pub.Close()

	// publish a message every second
	r := n.TimeRate(1 * time.Second)

	for {
		msg := &std_msgs.Bool{Data: true}
		fmt.Printf("Outgoing: %+v\n", msg)
		pub.Write(msg)

		r.Sleep()
	}
}

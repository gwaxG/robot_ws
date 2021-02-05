package controller

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/input"
	"github.com/gwaxG/robot_ws/control/pkg/output"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"net"
)

type Controller struct {
	connBase net.Conn
	connArm net.Conn
	node *goroslib.Node

	input []interface{}
	output interface{}
	imu interface{}
	manager state.Manager

	fromInput chan state.State
	toOutput chan state.State

}

// Initialize connections with the platform and roscore.
// This method automatically defines configuration of the control node
// based on availability of connections and keyboardSim flag provided at start time.
func (c * Controller) Init (keyboardSim bool) {
	c.connBase = connections.ConnectHostPort("192.168.0.60", "10001")
	c.connArm = connections.ConnectHostPort("192.168.0.60", "10001")
	c.node = connections.ConnectRos()

	// input sink to indicate what actions to execute
	c.fromInput = make(chan state.State)
	c.toOutput = make(chan state.State)

	if c.connBase != nil {
		c.input = append(c.input, input.Keyboard{})
		if c.node != nil {
			c.input = append(c.input, input.Ros{})
			c.output = output.PlatformCmd{}
			c.imu = output.PlatformImu{}
		}
	} else {
		if keyboardSim {
			c.input = append(c.input, input.Keyboard{})
		}
		c.input = append(c.input, input.Ros{})
		c.output = output.RosCmd{}
	}

	for _, s := range c.input {
		switch s.(type) {
		case input.Keyboard:
			s.(*input.Keyboard).Init(c.fromInput)
			go s.(*input.Keyboard).Serve()
		case input.Ros:
			s.(*input.Ros).Init(c.fromInput)
			go s.(*input.Ros).Serve()
		}
	}

	switch c.output.(type) {
	case output.PlatformCmd:
		c.output.(*output.PlatformCmd).Init(c.toOutput)
		go c.output.(*output.PlatformCmd).Serve()
	case output.RosCmd:
		c.output.(*output.RosCmd).Init(c.toOutput)
		go c.output.(*output.RosCmd).Serve()
	}
	if c.imu != nil {
		c.imu.(*output.PlatformImu).Init()
		go c.output.(*output.PlatformImu).Serve()
	}

	c.manager = state.Manager{}
}


func (c *Controller) Start () {
	st := state.State{}
	for {
		st = <- c.fromInput
		go c.manager.Monitor(&st)
		c.toOutput <- st
	}
}


func (c * Controller) Close(){
	if c.connBase != nil {
		err := c.connBase.Close()
		utils.FailOnError(err, "Failed to close tcp connection:base")
	}
	if c.connArm != nil {
		err := c.connArm.Close()
		utils.FailOnError(err, "Failed to close tcp connection:arm")
	}
	if c.node != nil {
		err := c.node.Close()
		utils.FailOnError(err, "Failed to close node")
	}
}
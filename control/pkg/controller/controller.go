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
	//keyboardCmdInput *input.Keyboard
	//rosCmdInput *input.Ros

	input interface{}
	output interface{}
	imu interface{}

	//platformCmdOutput *output.PlatformCmd
	//rosCmdOutput *output.RosCmd
	//platformImu *output.PlatformImu
	//keyboardCmdInput *input.Keyboard
	//rosCmdInput *input.Ros

	state state.State
	stateChange chan state.State

	connBase net.Conn
	connArm net.Conn
	node *goroslib.Node
}

// Initialize connections with the platform and roscore.
// This method automatically defines configuration of the control node
// based on availability of connections and keyboardSim flag provided at start time.
func (c * Controller) Init (keyboardSim bool) {

	c.connBase = connections.ConnectHostPort("192.168.0.60", "10001")
	c.connArm = connections.ConnectHostPort("192.168.0.60", "10001")
	c.node = connections.ConnectRos()
	c.stateChange = make(chan state.State)

	//platformCmdOutput *output.PlatformCmd
	//rosCmdOutput *output.RosCmd
	//platformImu *output.PlatformImu
	//keyboardCmdInput *input.Keyboard
	//rosCmdInput *input.Ros

	if c.connBase != nil {
		c.input = ""
		// c.keyboardCmdInput.Init(c.stateChange)
		if c.node != nil {
			c.rosCmdInput.Init(c.stateChange)
			c.platformCmdOutput.Init()
			c.platformImu.Init()
		}
	} else {
		if keyboardSim {
			c.keyboardCmdInput.Init(c.stateChange)
		}
		c.rosCmdInput.Init(c.stateChange)
		c.rosCmdOutput.Init()
	}
}


func (c *Controller) Start () {
	if c.keyboardCmdInput != nil {
		go c.keyboardCmdInput.Serve()
	}
	if c.rosCmdInput != nil {
		go c.rosCmdInput.Serve()
	}

	change := state.State{}

	for {
		change = <- c.stateChange
		if c.platformCmd != nil {
			go c.platformCmd.ApplyChange(change)
		}
		if c.rosCmd != nil {
			go c.rosCmd.ApplyChange(change)
		}
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
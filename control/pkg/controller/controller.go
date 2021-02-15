package controller

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/input"
	"github.com/gwaxG/robot_ws/control/pkg/output"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"log"
	"net"
)

type Controller struct {
	connBase net.Conn
	connArm net.Conn
	node *goroslib.Node

	input []interface{}
	output interface{}
	imu interface{}
	statePublisher interface{}
	manager state.Manager

	fromInput chan state.State
	toOutput chan state.State
	publishState chan state.State

}

// Initialize connections with the platform and roscore.
// This method automatically defines configuration of the control node
// based on availability of connections and keyboardSim flag provided at start time.
func (c * Controller) Init (inputKeyboard, inputRos, test, outputPlatform, outputSimulation bool) {
	log.Println("Initializing controller...")

	// input sink to route actions to execute
	c.fromInput = make(chan state.State)
	c.toOutput = make(chan state.State)
	c.publishState = make(chan state.State)

	// creating struct instances
	if inputKeyboard {
		log.Println("Keyboard enabled")
		c.input = append(c.input, &input.Keyboard{})
	}
	if inputRos {
		log.Println("ROS input enabled on /platform_cmd")
		c.node = connections.ConnectRos(outputSimulation)
		c.input = append(c.input, &input.Ros{})
	}
	if outputPlatform {
		log.Println("Platform output enabled")
		c.output = output.PlatformCmd{}
		c.imu = output.PlatformImu{}
	}
	if outputSimulation {
		log.Println("Simulation output enabled")
		if c.node == nil {
			c.node = connections.ConnectRos(outputSimulation)
		}
		c.output = output.RosCmd{}
	}

	// If ROS enabled, then we publish the robot state to /robot/state
	if inputRos || outputSimulation {
		log.Println("State publishing started")
		statePublisher := output.StatePublisher{}
		statePublisher.Init(c.node, c.publishState)
		go statePublisher.Publish()
	}

	if test {
		c.connBase = connections.ConnectHostPort("localhost", "10001")
		c.connArm = connections.ConnectHostPort("localhost", "10002")
	} else {
		c.connBase = connections.ConnectHostPort("192.168.0.60", "10001")
		c.connArm = connections.ConnectHostPort("192.168.0.63", "10001")
	}
	for _, s := range c.input {
		switch s.(type) {
		case *input.Keyboard:
			log.Println("Keyboard input started")
			s.(*input.Keyboard).Init(c.fromInput)
			go s.(*input.Keyboard).Serve()
		case *input.Ros:
			log.Println("ROS input started")
			s.(*input.Ros).Init(c.fromInput)
			go s.(*input.Ros).Serve()
		}
	}

	switch c.output.(type) {
	case *output.PlatformCmd:
		log.Println("Sending commands to the platform")
		c.output.(*output.PlatformCmd).Init(c.toOutput)
		go c.output.(*output.PlatformCmd).Serve()
	case *output.RosCmd:
		log.Println("Sending commands to ROS")
		c.output.(*output.RosCmd).Init(c.node, c.toOutput)
		go c.output.(*output.RosCmd).Serve()
	}

	if c.imu != nil {
		log.Println("platform IMU enabled")
		c.imu.(*output.PlatformImu).Init()
		go c.output.(*output.PlatformImu).Serve()
	}

	c.manager = state.Manager{}
	c.manager.Init()
}

func (c *Controller) Start () {
	actions := state.State{}
	state := state.State{}
	log.Println("Started...")
	for {
		log.Println("Start loop")
		actions = <- c.fromInput
		log.Println("Action", actions)
		state, actions = c.manager.Monitor(actions)
		log.Println("State actual", actions)
		log.Println("Action actual", actions)
		c.publishState <- state
		log.Println("Pre-end loop")
		c.toOutput <- actions
		log.Println("End loop")
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

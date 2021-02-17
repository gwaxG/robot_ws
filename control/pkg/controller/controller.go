package controller

import (
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/input"
	"github.com/gwaxG/robot_ws/control/pkg/output"
	"github.com/gwaxG/robot_ws/control/pkg/platform_sensors"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"log"
	"net"
)

type Controller struct {
	connBase 		net.Conn
	connArm 		net.Conn
	save 			func()()
	input 			[]interface{}
	output			interface{}
	imu 			interface{}
	statePublisher  interface{}
	manager 		state.Manager
	fromInput 		chan state.State
	toOutput 		chan []state.State
	publishState 	chan state.State
	done 			chan bool
	reset 			chan bool
	keyboardUsage 	chan bool
	stopReleaseCh   chan string
}

// Initialize connections with the platform and roscore.
// This method automatically defines configuration of the control node
// based on availability of connections and keyboardSim flag provided at start time.
func (c * Controller) Init (inputKeyboard, inputRos, test, outputPlatform, outputSimulation bool) {
	log.Println("Initializing controller...")

	// input sink to route actions to execute
	c.fromInput = make(chan state.State)
	c.toOutput = make(chan []state.State)
	c.publishState = make(chan state.State)
	c.done = make(chan bool)
	c.reset = make(chan bool)
	c.keyboardUsage = make(chan bool)

	// creating struct instances
	if inputKeyboard {
		c.input = append(c.input, &input.Keyboard{})
	}
	if inputRos {
		c.input = append(c.input, &input.Ros{})
	}
	if outputPlatform {
		c.stopReleaseCh = make(chan string)
		c.output = &output.PlatformCmd{}
		c.imu = &platform_sensors.PlatformSensors{}
	}
	if outputSimulation {
		c.output = &output.RosCmd{}
	}

	// If ROS enabled, then we publish the robot state to /robot/state
	if inputRos || outputSimulation {
		log.Println("State publishing started")
		statePublisher := output.StatePublisher{}
		statePublisher.Init(c.publishState)
		go statePublisher.Publish()
	}

	for _, s := range c.input {
		switch s.(type) {
		case *input.Keyboard:
			log.Println("Keyboard input started")
			s.(*input.Keyboard).Init(c.fromInput, c.reset, c.done, c.keyboardUsage, c.stopReleaseCh)
			go s.(*input.Keyboard).Serve()
		case *input.Ros:
			log.Println("ROS input started")
			s.(*input.Ros).Init(c.fromInput, c.reset)
			go s.(*input.Ros).Serve()
		}
	}

	switch c.output.(type) {
	case *output.PlatformCmd:
		log.Println("Platform output started")
		c.output.(*output.PlatformCmd).Init(c.stopReleaseCh, test, c.toOutput)
		go c.output.(*output.PlatformCmd).Serve()
	case *output.RosCmd:
		log.Println("Simulation output started")
		c.output.(*output.RosCmd).Init(c.toOutput)
		go c.output.(*output.RosCmd).Serve()
	}

	if c.imu != nil {
		log.Println("platform sensor module started")
		c.imu.(*platform_sensors.PlatformSensors).Init(c.stopReleaseCh)
		go c.output.(*platform_sensors.PlatformSensors).Serve()
	}

	c.manager = state.Manager{}
	c.manager.Init()

	// Saving robot state
	if outputPlatform {
		c.save = c.manager.Save
		StateAction, Change := c.manager.Load()
		c.publishState <- StateAction
		c.toOutput <- []state.State{StateAction, Change}
	}
}

func (c *Controller) Start () {
	var set, end bool
	var StateAction, Change state.State
	log.Println("Controller started...")

	// control workflow
	for {
		select {
		case <- c.reset:
			StateAction, Change = c.manager.Reset()
		case _ = <-c.done:
			end = true
		case actions := <- c.fromInput:
			select{
			case <- c.keyboardUsage:
				set = false
			default:
				set = true
			}
			StateAction, Change = c.manager.Monitor(set, actions)
		}
		if end {
			break
		} else {
			log.Println("StateAction ", StateAction)
			log.Println("Change ", Change)
			c.publishState <- StateAction
			c.toOutput <- []state.State{StateAction, Change}
			if c.save != nil {
				c.save()
			}
		}
	}
}



func (c * Controller) Close(){
	connections.Close()
}

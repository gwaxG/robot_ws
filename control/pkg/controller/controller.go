package controller

//import "github.com/gwaxG/robot_ws/control/pkg/utils"
import (
	"fmt"
	"github.com/gwaxG/robot_ws/control/pkg/input"
	"github.com/gwaxG/robot_ws/control/pkg/state"
)

type Controller struct {
	keyboard input.Keyboard
	ros input.Ros
	state state.State
}

func (c * Controller) Init () {
	// isPlatform, isRos := utils.Flags()
}

func (c *Controller) Start () {
	stateChange := make(chan state.State)
	// go c.keyboard.Serve(stateChange)
	go c.ros.Serve(stateChange)

	change := state.State{}

	for {
		change = <- stateChange
		c.applyChange(&change)
	}

}

func (c *Controller) applyChange(change *state.State) {
	fmt.Println("Applying the change", *change)
}
package core

import (
	"fmt"
	"github.com/gwaxG/robot_ws/backend/internal/ros"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"

)

type Core struct {
	ros			ros.Ros
	analyticsCh	chan structs.Analytics
}

func (c *Core) Init() {
	c.analyticsCh = make(chan structs.Analytics)
	c.ros = ros.Ros{}
	c.ros.Init(c.analyticsCh)
}

func (c *Core) Start() {
	var (
		analytics structs.Analytics
	)
	for {
		select {
			case analytics = <- c.analyticsCh:
				c.RegisterRollout(analytics)
		}
	}
}

func (c *Core) RegisterRollout(analytics structs.Analytics) {
		fmt.Println("Registering", analytics)
}

func (c *Core) Close() {
	c.ros.Close()
}


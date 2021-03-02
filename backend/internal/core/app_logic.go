package core

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/backend/internal/common"
	"github.com/gwaxG/robot_ws/backend/internal/database"
	"github.com/gwaxG/robot_ws/backend/internal/ros"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"

)

type Core struct {
	ros			ros.Ros
	database	database.DataBase
	analyticsCh	chan structs.Analytics
}

func (c *Core) Init() {
	c.analyticsCh = make(chan structs.Analytics)
	c.ros = ros.Ros{}
	c.ros.Init(c.analyticsCh)
	c.database = database.DataBase{}
	c.database.Init(c.ros.ExpSeriesName)
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
	c.database.Close()
}


package core

import (
	_ "github.com/aler9/goroslib"
	_ "github.com/gwaxG/robot_ws/backend/internal/common"
	"github.com/gwaxG/robot_ws/backend/internal/database"
	"github.com/gwaxG/robot_ws/backend/internal/ros"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
)

type Core struct {
	ros			ros.Ros
	database	database.DataBase
	analyticsCh	chan structs.RolloutAnalytics
}

func (c *Core) Init() {
	c.analyticsCh = make(chan structs.RolloutAnalytics)
	c.ros = ros.Ros{}
	c.ros.Init(c.analyticsCh)
	c.database = database.DataBase{}
	c.database.Init(c.ros.ExpSeriesName)
}

func (c *Core) Start() {
	var (
		analytics structs.RolloutAnalytics
	)
	for {
		select {
			case analytics = <- c.analyticsCh:
				c.database.AddNewRolloutAnalytics(analytics)
		}
	}
}

func (c *Core) Close() {
	c.ros.Close()
	c.database.Close()
}


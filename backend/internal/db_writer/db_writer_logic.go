package db_writer

import (
	"log"

	_ "github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	database2 "github.com/gwaxG/robot_ws/backend/pkg/database"
)

type Core struct {
	ros         Ros
	database    database2.DataBase
	analyticsCh chan common.RolloutAnalytics
}

func (c *Core) Init() {
	c.analyticsCh = make(chan common.RolloutAnalytics)
	c.ros = Ros{}
	c.ros.Init(c.analyticsCh)
	c.database = database2.DataBase{}
	c.database.Init()
}

func (c *Core) Start() {
	// Closing connections on error
	defer func() {
		if r := recover(); r != nil {
			log.Println("Recovered in Core.Start()", r)
			c.Close()
		}
	}()

	var analytics common.RolloutAnalytics

	for {
		select {
		// New analytics coming from ROS
		case analytics = <-c.analyticsCh:
			go c.database.AddNewRolloutAnalytics(analytics)
		}
	}
}

func (c *Core) Close() {
	c.ros.Close()
	c.database.Close()
}

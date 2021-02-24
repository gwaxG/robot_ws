package beam_features

import (
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"log"
)

type Core struct {
	ros RosProxy
}

func (c *Core) Init() {
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
}

func (c *Core) Start () {
	log.Println("Started")
	select {}
}

func (c *Core) Handle(img *sensor_msgs.Image) {
	log.Println("Image received")
}
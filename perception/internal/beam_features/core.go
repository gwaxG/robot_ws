package beam_features

import (
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
)

type Core struct {
	ros RosProxy
}

func (c *Core) Init() {
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
}

func (c *Core) Handle(img sensor_msgs.Image) {

}
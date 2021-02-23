package beam_features

import (
	"fmt"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"net/http"
)

type Core struct {
	ros RosProxy
}

func (c *Core) Init() {
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
}

func (c *Core) Start() {
	req, _ := http.NewRequest("GET", "http://api.themoviedb.org/3/tv/popular", nil)

	q := req.URL.Query()
	q.Add("api_key", "key_from_environment_or_flag")
	q.Add("another_thing", "foo & bar")
	req.URL.RawQuery = q.Encode()
	esp, _:=http.Get(req.URL.String())
	fmt.Println(esp)

	select {}
}

func (c *Core) Handle(img sensor_msgs.Image) {

}
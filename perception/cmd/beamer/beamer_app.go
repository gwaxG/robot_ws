package main

import (
	"github.com/gwaxG/robot_ws/perception/internal/beam_features"
	"os"
)

func main() {
	var port string
	if len(os.Args) == 1 {
		port = "11311"
	} else {
		port = os.Args[1]
	}
	os.Setenv("ROS_MASTER_URI", "http://localhost:"+port)

	core := beam_features.Core{}
	core.Init()
	core.Start()
}

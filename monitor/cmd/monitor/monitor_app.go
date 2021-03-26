package main

import (
	"github.com/gwaxG/robot_ws/monitor/internal/monitor"
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

	app := monitor.Core{}
	defer app.Close()
	app.Init()
	app.Start()
}

package main

import (
	"flag"
	"os"

	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	/*
		TODO
		test real platform
	*/
	var port string
	if len(os.Args) == 1 {
		port = "11311"
	} else {
		port = os.Args[1]
	}
	os.Setenv("ROS_MASTER_URI", "http://localhost:"+port)

	iKeyb := flag.Bool("ik", true, "Keyboard input")                     // true
	iRos := flag.Bool("ir", true, "ROS input")                           // true
	test := flag.Bool("t", false, "Test IP:HOST platform configuration") // false
	oPlat := flag.Bool("op", false, "Platform command output")            // false
	oSim := flag.Bool("os", true, "Simulation command output")          // true
	flag.Parse()

	c := controller.Controller{}
	c.Init(*iKeyb, *iRos, *test, *oPlat, *oSim)
	c.Start()
	c.Close()
}

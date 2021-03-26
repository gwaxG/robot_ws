package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/control/pkg/controller"
	"os"
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

	iKeyb := flag.Bool("ik", true, "Keyboard input")
	iRos := flag.Bool("ir", true, "ROS input")
	test := flag.Bool("t", false, "Test IP:HOST platform configuration")
	oPlat := flag.Bool("op", false, "Platform command output")
	oSim := flag.Bool("os", true, "Simulation command output")
	flag.Parse()

	c := controller.Controller{}
	c.Init(*iKeyb, *iRos, *test, *oPlat, *oSim)
	c.Start()
	c.Close()
}

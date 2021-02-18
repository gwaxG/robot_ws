package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	/*
	TODO
	test platform
	*/
	
	iKeyb := flag.Bool("ik", true, "Keyboard input")
	iRos := flag.Bool("ir", false, "ROS input")
	test := flag.Bool("t", true, "Test IP:HOST platform configuration")
	oPlat := flag.Bool("op", true, "Platform command output")
	oSim := flag.Bool("os", false, "Simulation command output")
	flag.Parse()

	c := controller.Controller{}
	c.Init(*iKeyb, *iRos, *test, *oPlat, *oSim)
	c.Start()
	c.Close()
}

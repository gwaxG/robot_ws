package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	/*
	TODO
	Init state simulation
	Output to the platform
	Output of platform IMU data
	*/
	
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

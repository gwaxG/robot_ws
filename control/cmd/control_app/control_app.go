package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	/*
	TODO
	Output to ROS
	Output to the platform
	Output of platform IMU data
	*/
	
	keyboardSim := flag.Bool("k", false, "Simulation keyboard usage")
	test := flag.Bool("t", true, "test platform")
	flag.Parse()

	c := controller.Controller{}
	c.Init(*keyboardSim, *test)
	c.Start()
}

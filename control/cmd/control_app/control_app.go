package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	keyboardSim := flag.Bool("k", false, "Simulation keyboard usage")
	flag.Parse()

	c := controller.Controller{}
	c.Init(*keyboardSim)
	c.Start()
}

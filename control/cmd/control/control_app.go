package main

import (
	"flag"
	"fmt"
	"os"
	"strings"

	"github.com/gwaxG/robot_ws/control/pkg/controller"
)

func main() {
	defer func() {
		if r := recover(); r != nil {
			if cont := strings.Contains(r.(string), "send on closed channel"); cont {
				// Ignore, because the error happened on exit due to the goroslib library.
			}
		}
	}()
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
	oPlat := flag.Bool("op", false, "Platform command output")           // false
	oSim := flag.Bool("os", true, "Simulation command output")           // true
	flag.Parse()
	fmt.Println(*iKeyb, *iRos, *test, *oPlat, *oSim)
	c := controller.Controller{}
	c.Init(*iKeyb, *iRos, *test, *oPlat, *oSim)
	c.Start()
	c.Close()
}

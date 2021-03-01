package main

import "github.com/gwaxG/robot_ws/monitor/internal/monitor"

func main() {
	app := monitor.Core{}
	defer app.Close()
	app.Init()
	app.Start()
}

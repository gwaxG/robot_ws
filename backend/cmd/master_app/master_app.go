package main

import "github.com/gwaxG/robot_ws/backend/internal/master"

func main() {
	app := master.Server{}
	defer app.Close()
	app.Init()
	app.Start()
}

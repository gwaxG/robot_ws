package main

import core "github.com/gwaxG/robot_ws/backend/internal/master"

func main() {
	app := core.Core{}
	defer app.Close()
	app.Init()
	app.Start()
}

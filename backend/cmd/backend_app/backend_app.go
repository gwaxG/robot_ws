package main

import (
	"github.com/gwaxG/robot_ws/backend/internal/core"
)

func main() {
	app := core.Core{}
	defer app.Close()
	app.Init()
	app.Start()
}
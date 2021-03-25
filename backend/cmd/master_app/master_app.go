package main

import (
	"flag"
	"github.com/gwaxG/robot_ws/backend/internal/master"
)

func main() {
	poolSize := flag.Int("psize", 1, "Active worker pool size")
	flag.Parse()
	app := master.Server{}
	defer app.Close()
	app.Init(uint8(*poolSize))
	app.Start()
}

package main

import (
	"github.com/gwaxG/robot_ws/perception/internal/beam_features"
)

func main() {
	core := beam_features.Core{}
	core.Init()
	core.Start()
}

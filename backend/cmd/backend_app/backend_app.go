package backend_app

import (
	core "github.com/gwaxG/robot_ws/backend/internal/backend"
)

func main() {
	app := core.Core{}
	defer app.Close()
	app.Init()
	app.Start()
}
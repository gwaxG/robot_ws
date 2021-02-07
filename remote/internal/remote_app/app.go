package remote_app

import "github.com/gin-gonic/gin"

type App struct {
	ros rosClient
}

func (a *App) Test() {
	ros := rosClient{}
	ros.init()
}

func (a *App) init() {
	a.ros = rosClient{}
	a.ros.init()
}

func (a *App) Start() {
	a.init()

	r := gin.Default()
	r.GET("/add/task", a.addTask)
	r.GET("/get/conf", a.getTaskConf)
	r.GET("/get/worker_state", a.getWorkerState)
	r.GET("/get/done_tasks", a.getDoneTasks)
	r.GET("/get/curves", a.getCurves)
	r.Run() // listen and serve on 0.0.0.0:8080 (for windows "localhost:8080")
}
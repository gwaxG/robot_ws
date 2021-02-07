package remote_app

import (
	// "encoding/json"
	"github.com/gin-gonic/gin"
)


func (a *App) getTaskConf(c *gin.Context) {
	a.ros.getConf()
	// conf, err := json.Marshal(a.ros.task)
	// FailedOnError(err, "can not get the task")
	c.JSON(200, a.ros.task)
}

func (a *App) getWorkerState(c *gin.Context) {}
func (a *App) getDoneTasks(c *gin.Context) {}
func (a *App) getCurves(c *gin.Context) {}

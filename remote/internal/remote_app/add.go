package remote_app

import (
	"encoding/json"
	"github.com/gin-gonic/gin"
	"io/ioutil"
	"log"
)

func (a *App) addTask(c *gin.Context) {
	var resp map[string]interface{}
	jsonData, _ := ioutil.ReadAll(c.Request.Body)
	_ = json.Unmarshal(jsonData, &resp)
	log.Println("Received!", resp)
	// ...
}

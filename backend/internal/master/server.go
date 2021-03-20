package master

import (
	"encoding/json"
	"github.com/gin-gonic/gin"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"github.com/gwaxG/robot_ws/backend/pkg/database"
	"io"
	"io/ioutil"
	"log"
	"net/http"
)

type Server struct {
	api  		*gin.Engine
	db 			database.DataBase
	launcher 	Launcher
}

// Core module organizes communication between UI (server), database (rpc) and learning env. (launcher).
func (s *Server) Init(poolSize uint8) {
	s.api = gin.Default()
	s.InitAPI()
	// database
	s.db = database.DataBase{}
	s.db.Init()
	// launcher
	s.launcher = Launcher{}
	s.launcher.Init(poolSize, &s.db)
}

func (s *Server) Start() {
	go s.launcher.Start()
	s.api.Run(":10000")
}

func (s *Server) Close() {
	s.db.Close()
}

func (s *Server) InitAPI(){
	s.api.Use(s.CORSMiddleware())
	s.api.Use(s.JSONMiddleware())
	s.api.GET("/dbs", s.listDbs)
	s.api.GET("/colls", s.listColls)
	s.api.GET("/visualize", s.visualize)
	s.api.GET("/hist", s.fetchHistoryConfigs)
	s.api.GET("/configs", s.getConfigs)
	s.api.GET("/queue", s.getQueue)
	s.api.GET("/pool", s.getPool)
	s.api.POST("/task/create", s.createTask)
	s.api.OPTIONS("/task/create", s.preflight)
	s.api.POST("/task/update", s.updateTask)
	s.api.OPTIONS("/task/update", s.preflight)
	s.api.POST("/task/delete", s.deleteTask)
	s.api.OPTIONS("/task/delete", s.preflight)

}

// List all databases
func (s *Server) listDbs(c *gin.Context) {
	data, err := s.db.FetchDbs()
	formJson(data, err, c)
}

// List a database collection with corresponding fields of the first entity
func (s *Server) listColls(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchColls(dbName)
	formJson(data, err, c)
}

// List a database collection history collection where all configs of passed experiments are stored
func (s *Server) fetchHistoryConfigs(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchHistoryConfig(dbName)
	formJson(data, err, c)
}

// Draw figures based on fields from a database collection
func (s *Server) visualize(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	collName := c.Query("collection") // shortcut for c.Request.URL.Query().Get("lastname")
	fields := c.Query("fields") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchVisualize(dbName, collName, fields)
	formJson(data, err, c)
}

// Get template algorithm configs
func (s *Server) getConfigs(c *gin.Context) {
	data, err := s.launcher.GetConfigs("")
	formJson(data, err, c)
}

// List  waiting tasks
func (s *Server) getQueue(c *gin.Context) {
	data, err := s.launcher.GetQueue()
	formJson(data, err, c)
}

// List being executed tasks
func (s *Server) getPool(c *gin.Context) {
	data, err := s.launcher.GetPool()
	formJson(data, err, c)
}

// 	CRUD queue tasks.
// You can not change being executed tasks.
// view task queue, add task to queue, delete task from queue, update task in queue
func (s *Server) createTask(c *gin.Context) {
	data, err := s.launcher.CreateTask(c.Request.Body)
	formJson(data, err, c)
}

func (s *Server) updateTask(c *gin.Context) {
	data, err := s.launcher.UpdateTask(c.Request.Body)
	formJson(data, err, c)
}

func (s *Server) deleteTask(c *gin.Context) {
	data, err := s.launcher.DeleteTask(c.Request.Body)
	formJson(data, err, c)
}

func formJson(data interface{}, err error, c *gin.Context) {
	if err == nil {
		c.JSON(http.StatusOK, data)
	} else {
		log.Println(err)
		c.JSON(http.StatusInternalServerError, nil)
	}
}

func retrieveJson(body io.Reader) map[string]interface{} {
	jsonBytes, err := ioutil.ReadAll(body)
	common.FailOnError(err)
	jsonData := map[string]interface{}{}
	err = json.Unmarshal(jsonBytes, &jsonData)
	common.FailOnError(err)
	return jsonData
}

func (s *Server) JSONMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Writer.Header().Set("Content-Type", "application/json")
		c.Next()
	}
}

func (s *Server) CORSMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Writer.Header().Set("Access-Control-Allow-Origin", "*")
		c.Next()
	}
}

func (s* Server) preflight(c *gin.Context) {
	c.Header("Access-Control-Allow-Origin", "*")
	c.Header(
		"Access-Control-Allow-Headers",
		"access-control-allow-origin, access-control-allow-headers, content-type")
	c.JSON(http.StatusOK, struct{}{})
}

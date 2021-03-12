package master

import (
	"github.com/gin-gonic/gin"
	"log"
	"net/http"
	"github.com/gwaxG/robot_ws/backend/pkg/database"
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
	s.launcher.Init(poolSize)
}

func (s *Server) Start() {
	s.api.Run(":10000")
}

func (s *Server) Close() {
	s.db.Close()
}

func (s *Server) InitAPI(){
	s.api.GET("/dbs", s.listDbs) // +
	s.api.GET("/colls", s.listColls) //
	s.api.GET("/visualize", s.visualize)
	s.api.GET("/configs", s.getConfigs)
	s.api.GET("/queue", s.listQueue)
	s.api.GET("/task/:config/*action", s.crudTask)
}

// List all databases
func (s *Server) listDbs(c *gin.Context) {
	data, err := s.db.FetchDbs()
	s.formJson(data, err, c)
}

// List a database collection with corresponding fields of the first entity
func (s *Server) listColls(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchColls(dbName)
	s.formJson(data, err, c)
}

// Draw figures based on fields from a database collection
func (s *Server) visualize(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	collName := c.Query("collection") // shortcut for c.Request.URL.Query().Get("lastname")
	fields := c.Query("fields") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchVisualize(dbName, collName, fields)
	s.formJson(data, err, c)
}

// Get template algorithm configs
func (s *Server) getConfigs(c *gin.Context) {
	data, err := s.launcher.GetConfigs()
	s.formJson(data, err, c)
}

// 	CRUD queue tasks.
// You can not change being executed tasks.
// view task queue, add task to queue, delete task from queue, update task in queue
func (s *Server) createTask(c *gin.Context) {
	data, err := s.launcher.CreateTask(...)
	s.formJson(data, err, c)
}

func (s *Server) readTask(c *gin.Context) {
	task := c.Param("task")
	data, err := s.launcher.ReadTask(task)
	s.formJson(data, err, c)
}

func (s *Server) updateTask(c *gin.Context) {
	task := c.Param("task")
	data, err := s.launcher.UpdateTask(task)
	s.formJson(data, err, c)
}

func (s *Server) deleteTask(c *gin.Context) {
	task := c.Param("task")
	data, err := s.launcher.DeleteTask(task)
	s.formJson(data, err, c)
}

// List being executed tasks and waiting tasks
func (s *Server) listQueue(c *gin.Context) {
	data, err := s.launcher.ListQueue()
	s.formJson(data, err, c)
}

func (s *Server) formJson(data interface{}, err error, c *gin.Context) {
	if err == nil {
		c.JSON(http.StatusOK, data)
	} else {
		log.Println(err)
		c.JSON(http.StatusInternalServerError, nil)
	}
}
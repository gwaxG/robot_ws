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
func (s *Server) Init() {
	s.api = gin.Default()
	s.InitAPI()
	// database
	s.db = database.DataBase{}
	s.db.Init()
	// launcher
	s.launcher = Launcher{}
	s.launcher.Init()
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
	s.api.GET("/queue", s.listQueue)
	s.api.GET("/configs", s.getConfigs)
	s.api.GET("/tasks", s.crudTasks)
}

// List all databases
func (s *Server) listDbs(c *gin.Context) {
	data, err := s.db.FetchDbs()
	if err == nil {
		c.JSON(200, data)
	} else {
		log.Println(err)
		c.JSON(http.StatusInternalServerError, nil)
	}
}

// List a database collection with corresponding fields of the first entity
func (s *Server) listColls(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	colls, err := s.db.FetchColls(dbName)
	if err == nil {
		c.JSON(200, colls)
	} else {
		log.Println(err)
		c.JSON(http.StatusInternalServerError, nil)
	}
}

// Draw figures based on fields from a database collection
func (s *Server) visualize(c *gin.Context) {
	dbName := c.Query("database") // shortcut for c.Request.URL.Query().Get("lastname")
	collName := c.Query("collection") // shortcut for c.Request.URL.Query().Get("lastname")
	fields := c.Query("fields") // shortcut for c.Request.URL.Query().Get("lastname")
	data, err := s.db.FetchVisualize(dbName, collName, fields)
	if err == nil {
		c.JSON(200, data)
	} else {
		log.Println(err)
		c.JSON(http.StatusInternalServerError, nil)
	}

}

// Get template algorithm configs
func (s *Server) getConfigs(c *gin.Context) {
	c.JSON(200, map[string]interface{}{})
}

// 	CRUD queue tasks.
// You can not change being executed tasks.
func (s *Server) crudTasks(c *gin.Context) {
	c.JSON(200, map[string]interface{}{})
}

// List being executed tasks and waiting tasks
func (s *Server) listQueue(c *gin.Context) {
	c.JSON(200, map[string]interface{}{})
}
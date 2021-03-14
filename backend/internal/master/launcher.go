package master

import (
	"encoding/json"
	"github.com/gin-gonic/gin"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"os"
	"path/filepath"
	"strings"
)

type Launcher struct {
	ConfigPool 	[]map[string]interface{} 	`json:"config_pool"`
	LaunchFiles []string 						`json:"launch_files"`
	PoolSize	uint8					 	`json:"pool_size"`
}

func (l *Launcher) Init(poolSize uint8){
	l.PoolSize = poolSize
	l.ConfigPool = []map[string]interface{}{}
}

type ResponseGetConfigs struct {
	Configs		[]map[string]interface{}	`json:"configs"`
	LaunchFiles	[]string					`json:"launch_files"`
	Msg			string 						`json:"msg"`
}

// List configs in scripts/learning_scripts
func (l *Launcher) GetConfigs() (ResponseGetConfigs, error){
	resp := ResponseGetConfigs{}
	var templates []string
	// list dir scripts/learning_scripts
	dir, err := filepath.Abs(filepath.Dir(os.Args[0]))
	common.FailOnError(err)
	dir, _ = filepath.Split(dir)
	dir = dir [:len(dir)-1]
	dir = filepath.Join(dir, "scripts", "learning_scripts")
	// keep only templates
	files, err := os.ReadDir(dir)
	common.FailOnError(err)
	for _, dirEntry := range files {
		// fmt.Println(dirEntry.Info())
		if strings.Contains(dirEntry.Name(), "template") {
			templates = append(templates, dirEntry.Name())
		}
	}
	// read templates to map[string]interface{}
	for _, template := range templates {
		config := map[string]interface{}{}
		f, err := os.ReadFile(filepath.Join(dir, template))
		common.FailOnError(err)
		err = json.Unmarshal(f, &config)
		resp.Configs = append(resp.Configs, config)
		resp.LaunchFiles = append(resp.LaunchFiles, filepath.Join(dir, strings.Replace(template, "_template.json", "_launch.py", 1)))
	}
	return resp, nil
}

type ResponseGetQueue struct {
	ConfigPool	[]map[string]interface{}	`json:"pool"`
	PoolSize 	uint8						`json:"pool_size"`
	LaunchFiles []string 					`json:"launch_files"`
	Msg			string 						`json:"msg"`
}

// returns the Launcher instance
func (l *Launcher) GetQueue() (ResponseGetQueue, error) {
	resp := ResponseGetQueue{}
	resp.ConfigPool = l.ConfigPool
	resp.PoolSize = l.PoolSize
	resp.LaunchFiles = l.LaunchFiles
	return resp, nil
}

type ResponseCreateTask struct {
	Config 		map[string]interface{}  `json:"config"`
	LaunchFile	string 					`json:"launch_file"`
	Msg 		string 					`json:"msg"`
}

func (l *Launcher) CreateTask(c *gin.Context) (ResponseCreateTask, error){
	// GOGOGO TODO
	resp := ResponseCreateTask{}
	return resp, nil
}

type ResponseReadTask struct {}

func (l *Launcher) ReadTask(task uint8) (ResponseReadTask, error){
	resp := ResponseReadTask{}
	return resp, nil
}

type ResponseUpdateTask struct {}

func (l *Launcher) UpdateTask(task uint8, taskDescr map[string]interface{}) (ResponseUpdateTask, error){
	resp := ResponseUpdateTask{}
	return resp, nil
}

type ResponseDeleteTask struct {}

func (l *Launcher) DeleteTask(task uint8) (ResponseDeleteTask, error){
	resp := ResponseDeleteTask{}
	return resp, nil
}

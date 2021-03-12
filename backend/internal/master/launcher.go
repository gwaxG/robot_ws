package master

import (
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"log"
	"os"
	"path"
	"path/filepath"
	"runtime"
)

type Launcher struct {
	ConfigPool 	[]map[string]interface{} 	`json:"config_pool"`
	PoolSize	uint8					`json:"pool_size"`
}

func getConfigPath() string {
	_, b, _, _ := runtime.Caller(0)
	// master
	basepath   := path.Dir(b)
	// internal
	basepath   = path.Dir(basepath)
	// module
	basepath   = path.Dir(basepath)
	return path.Join(basepath, "scripts/learning_scripts")
}

func (l *Launcher) Init(poolSize uint8){
	l.PoolSize = poolSize
	l.ConfigPool = []map[string]interface{}{}
}

type ResponseGetConfigs struct {
	Configs	[]map[string]interface{}	`json:"configs"`
	Msg		string 						`json:"msg"`
}

func (l *Launcher) GetConfigs() (ResponseGetConfigs, error){
	resp := ResponseGetConfigs{}
	// list dir scripts/learning_scripts
	dir, err := filepath.Abs(filepath.Dir(os.Args[0]))
	common.FailOnError(err)
	fmt.Prin
	// keep only templates

	// read templates to map[string]interface{}

	return resp, nil
}

type ResponseGetQueue struct {}

func (l *Launcher) GetQueue() (ResponseGetQueue, error) {
	resp := ResponseGetQueue{}
	return resp, nil
}


type ResponseCreateTask struct {}

func (l *Launcher) CreateTask(taskDescr map[string]interface{}) (ResponseCreateTask, error){
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

package master

import (
	"encoding/json"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"io"
	"io/ioutil"
	"os"
	"path/filepath"
	"strings"
	"time"
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



// start python script
func (l *Launcher) worker(jobs <-chan int, results chan<- bool) {
	// start environment

	// wait for jobs
	for _ = range jobs {
		time.Sleep(time.Duration(time.Second))
		results <- true
	}
}

func (l *Launcher) Start(){
	// create worker pool
	jobs := make(chan int, l.PoolSize)
	results := make(chan bool, l.PoolSize)
	for i := 0; i < int(l.PoolSize); i++ {
		go l.worker(jobs, results)
	}
	// endlessly check for new tasks
	for {

	}
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

type RequestCreateTask struct {
	Config 		map[string]interface{}  `json:"config"`
	LaunchFile	string 					`json:"launch_file"`
	Msg 		string 					`json:"msg"`
}

type ResponseCreateTask struct {
	Msg 		string 					`json:"msg"`
}

// Add a task to the config
func (l *Launcher) CreateTask(reqRaw io.ReadCloser) (ResponseCreateTask, error){
	req := RequestCreateTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)
	l.ConfigPool = append(l.ConfigPool, req.Config)
	l.LaunchFiles = append(l.LaunchFiles, req.LaunchFile)
	return ResponseCreateTask{}, nil
}


type RequestReadTask struct {
	TaskId 		int  					`json:"task_id"`
}

type ResponseReadTask struct {
	Config 		map[string]interface{}  `json:"config"`
	LaunchFile	string 					`json:"launch_file"`
	Found		bool					`json:"found"`
	Msg 		string 					`json:"msg"`
}

func (l *Launcher) ReadTask(reqRaw io.ReadCloser) (ResponseReadTask, error){
	req := RequestReadTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)

	taskId := req.TaskId

	resp := ResponseReadTask{}
	if len(l.ConfigPool) < taskId {
		resp.Found = false
		resp.Msg   = "taskId is higher than queue length"
		return resp, nil
	}
	resp.Config = l.ConfigPool[taskId]
	resp.LaunchFile = l.LaunchFiles[taskId]
	resp.Found = true
	return resp, nil
}

type RequestUpdateTask struct {
	Config 		map[string]interface{}  `json:"config"`
	TaskId 		int  					`json:"task_id"`
}

type ResponseUpdateTask struct {
	Found		bool					`json:"found"`
	Msg 		string 					`json:"msg"`
}

func (l *Launcher) UpdateTask(reqRaw io.ReadCloser) (ResponseUpdateTask, error){
	req := RequestUpdateTask{}
	resp := ResponseUpdateTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)

	taskId := req.TaskId

	// check for correct number of requested task id update
	if len(l.ConfigPool) < taskId {
		resp.Found = false
		resp.Msg   = "taskId is higher than queue length"
		return resp, nil
	}

	// check for correctness of requested config
	var keysPoolConf []string
	for k, _ := range l.ConfigPool[taskId] {
		keysPoolConf = append(keysPoolConf, k)
	}
	for k, _ := range req.Config {
		if !contains(keysPoolConf, k) {
			resp.Found = false
			resp.Msg = "configuration keys are not the same"
			return resp, nil
		}
	}

	l.ConfigPool[taskId] = req.Config
	resp.Found = true
	return resp, nil
}

type RequestDeleteTask struct {
	TaskId 		int  					`json:"task_id"`
}

type ResponseDeleteTask struct {
	Found		bool					`json:"found"`
	Msg 		string 					`json:"msg"`
}

func (l *Launcher) DeleteTask(reqRaw io.ReadCloser) (ResponseDeleteTask, error){
	req := RequestDeleteTask{}
	resp := ResponseDeleteTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)

	taskId := req.TaskId

	// check for correct number of requested task id update
	if len(l.ConfigPool) < taskId {
		resp.Found = false
		resp.Msg   = "taskId is higher than queue length"
		return resp, nil
	}

	var result bool
	l.ConfigPool, result = deleteMapStrInt(l.ConfigPool, taskId)
	if result == false {
		resp.Found = false
		resp.Msg   = "could not delete from config pool"
		return resp, nil
	}

	l.LaunchFiles, result = deleteStrings(l.LaunchFiles, taskId)
	if result == false {
		resp.Found = false
		resp.Msg   = "could not delete from launch files"
		return resp, nil
	}

	resp.Found = true
	return resp, nil
}

// custom contains function
func contains(s []string, e string) bool {
	for _, a := range s {
		if a == e {
			return true
		}
	}
	return false
}

// custom delete from map[string]interface{} by index function
func deleteMapStrInt(a []map[string]interface{}, i int) ([]map[string]interface{}, bool){
	defer func (){
		if r:=recover(); r!=nil{}
	}()
	var aNew []map[string]interface{}
	aNew = a[:i]
	aNew = append(aNew, a[i+1:]...)
	return aNew, true
}

// custom delete from string array by index function
func deleteStrings(a []string, i int) ([]string, bool){
	defer func (){
		if r:=recover(); r!=nil{}
	}()
	var aNew []string
	aNew = a[:i]
	aNew = append(aNew, a[i+1:]...)
	return aNew, true
}
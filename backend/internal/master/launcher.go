package master

import (
	"encoding/json"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"io"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"strings"
	"time"
)

type Launcher struct {
	// wait queue
	WaitQueue 	[]map[string]interface{} 	`json:"config_pool"`
	WaitLaunchFiles []string 				`json:"launch_files"`
	// worker pool
	ActivePool 	map[int]*Job 				`json:"active_pool"`
	PoolSize	uint8					 	`json:"pool_size"`
	// workers
	NeedToCheck	chan bool
}



func (l *Launcher) Init(poolSize uint8){
	l.WaitQueue = []map[string]interface{}{}
	l.WaitLaunchFiles = []string{}
	l.ActivePool = map[int]*Job{}
	for i := 0; i<int(poolSize); i++ {
		l.ActivePool[i] = nil
	}
	l.PoolSize = poolSize
	l.NeedToCheck = make(chan bool)
}


type Job struct {
	Config 		map[string]interface{}		`json:"config"`
	WorkerId	string						`json:"worker_id"`
	LaunchFile	string						`json:"launch_file"`
}

// start python script
func (l *Launcher) worker(id int, jobs <-chan Job) {
	var job Job
	// start environment
	/* port := 11311 + id
	os.Setenv("ROS_MASTER_URI", "http://localhost:"+strconv.Itoa(port))
	cmd := exec.Command("roslaunch", "-p", strconv.Itoa(port), "backend", "learning.launch")
	if err := cmd.Start(); err != nil {
		log.Fatal(err)
	} */
	// start serving
	for job = range jobs {
		l.ActivePool[id] = &job
		// start job
		time.Sleep(time.Second*1)
		/* cmd = exec.Command("python", job.LaunchFile)
		if err := cmd.Start(); err != nil {
			log.Fatal(err)
		}*/

		l.ActivePool[id] = nil
		l.NeedToCheck <- true
	}
}

func (l *Launcher) checkPool() bool {
	for _, v := range l.ActivePool {
		if v == nil {
			return true
		}
	}
	return false
}


func (l *Launcher) Start(){
	// create worker pool
	jobs := make(chan Job, l.PoolSize)
	for id := 0; id < int(l.PoolSize); id++ {
		go l.worker(id, jobs)
	}
	// trigger
	for {
		select  {
			case _ = <- l.NeedToCheck:
				empty := l.checkPool()
				if len(l.WaitQueue) > 0 && empty{
					log.Println("Trigger")
					job := Job{
						Config: l.WaitQueue[0],
						LaunchFile: l.WaitLaunchFiles[0],
					}
					l.WaitQueue, _ = deleteMapStrInt(l.WaitQueue, 0)
					l.WaitLaunchFiles, _ = deleteStrings(l.WaitLaunchFiles, 0)
					jobs <- job
					go func (){l.NeedToCheck <- true}()
				}
		}
	}
}

type ResponseGetConfigs struct {
	Configs			[]map[string]interface{}	`json:"configs"`
	WaitLaunchFiles	[]string					`json:"launch_files"`
	Msg				string 						`json:"msg"`
}

// List configs in scripts/learning_scripts
func (l *Launcher) GetConfigs(pat string) (ResponseGetConfigs, error){
	if pat == "" {
		pat = "template"
	}
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
		if strings.Contains(dirEntry.Name(), pat) {
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
		resp.WaitLaunchFiles = append(resp.WaitLaunchFiles, filepath.Join(dir, strings.Replace(template, "_template.json", "_launch.py", 1)))
	}
	return resp, nil
}

type ResponseGetQueue struct {
	Queue		[]map[string]interface{}	`json:"queue"`
	LaunchFiles []string 					`json:"launch_files"`
	PoolSize 	uint8						`json:"pool_size"`
	Msg			string 						`json:"msg"`
}

// return wait queue
func (l *Launcher) GetQueue() (ResponseGetQueue, error) {
	resp := ResponseGetQueue{}
	resp.Queue = append(resp.Queue, l.WaitQueue...)
	resp.LaunchFiles = append(resp.LaunchFiles, l.WaitLaunchFiles...)
	resp.PoolSize = l.PoolSize
	return resp, nil
}

type ResponseGetPool struct {
	Pool		[]map[string]interface{}	`json:"pool"`
	LaunchFiles []string 					`json:"launch_files"`
	Msg			string 						`json:"msg"`
}

// return active pool
func (l *Launcher) GetPool() (ResponseGetPool, error) {
	resp := ResponseGetPool{}

	for _, job := range l.ActivePool {
		if job != nil {
			resp.Pool = append(resp.Pool, job.Config)
			resp.LaunchFiles = append(resp.LaunchFiles, job.LaunchFile)
		}
	}
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

	l.WaitQueue = append(l.WaitQueue, req.Config)
	l.WaitLaunchFiles = append(l.WaitLaunchFiles, req.LaunchFile)
	l.NeedToCheck <- true
	return ResponseCreateTask{}, nil
}



type RequestUpdateTask struct {
	Config 		map[string]interface{}  `json:"config"`
	TaskId 		int  					`json:"task_id"`
}

type ResponseUpdateTask struct {
	Found		bool					`json:"found"`
	Msg 		string 					`json:"msg"`
}

// update task config in wait queue
func (l *Launcher) UpdateTask(reqRaw io.ReadCloser) (ResponseUpdateTask, error){
	req := RequestUpdateTask{}
	resp := ResponseUpdateTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)

	taskId := req.TaskId

	// check for correct number of requested task id update
	if len(l.WaitQueue) < taskId {
		resp.Found = false
		resp.Msg   = "taskId is higher than queue length"
		return resp, nil
	}

	// check for correctness of requested config
	keysPoolConf := []string{}

	for k, _ := range l.WaitQueue[taskId] {
		keysPoolConf = append(keysPoolConf, k)
	}
	for k, _ := range req.Config {
		if !contains(keysPoolConf, k) {
			resp.Found = false
			resp.Msg = "configuration keys are not the same"
			return resp, nil
		}
	}

	l.WaitQueue[taskId] = req.Config
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
	if len(l.WaitQueue) < taskId {
		resp.Found = false
		resp.Msg   = "taskId is higher than queue length"
		return resp, nil
	}

	var result bool
	l.WaitQueue, result = deleteMapStrInt(l.WaitQueue, taskId)
	if result == false {
		resp.Found = false
		resp.Msg   = "could not delete from config pool"
		return resp, nil
	}

	l.WaitLaunchFiles, result = deleteStrings(l.WaitLaunchFiles, taskId)
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
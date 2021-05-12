package master

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"github.com/gwaxG/robot_ws/backend/pkg/database"
)

type Launcher struct {
	// wait queue
	WaitQueue       []map[string]interface{} `json:"config_pool"`
	WaitLaunchFiles []string                 `json:"launch_files"`
	// worker pool
	ActivePool map[int]*Job `json:"active_pool"`
	PoolSize   uint8        `json:"pool_size"`
	db         *database.DataBase
	// workers
	NeedToCheck      chan bool
	Done             chan struct{}
	RestartContainer chan int
}

func (l *Launcher) Init(poolSize uint8, db *database.DataBase) {
	l.db = db
	l.RestartContainer = make(chan int)
	l.WaitQueue = []map[string]interface{}{}
	l.WaitLaunchFiles = []string{}
	l.ActivePool = map[int]*Job{}
	for i := 0; i < int(poolSize); i++ {
		l.ActivePool[i] = nil
	}
	l.PoolSize = poolSize
	l.NeedToCheck = make(chan bool)
}

type Job struct {
	Config     map[string]interface{} `json:"config"`
	WorkerId   int                    `json:"worker_id"`
	LaunchFile string                 `json:"launch_file"`
	TaskId     int                    `json:"task_id"`
}

// Launch a learning script.
func (j *Job) Do(port int) (e error) {
	defer func() {
		if r := recover(); r != nil {
			err := r.(error)
			e = fmt.Errorf("Something went wrong in job %d on port %d: %s\n", j.TaskId, port, err)
		}
	}()
	// Form config file path.
	log.Printf("Job %d on port %d starting scrpit\n", j.TaskId, port)
	base := strings.Replace(path.Base(j.LaunchFile), ".py", ".json", 1)
	configPath := path.Join(path.Dir(j.LaunchFile), base)
	// Dump config map[string]interface{} into the json config file.
	marshalled, err := json.Marshal(j.Config)
	if err != nil {
		return err
	}
	err = ioutil.WriteFile(configPath, marshalled, 0644)
	log.Printf("Job %d on port %d creating launch config\n", j.TaskId, port)
	if err != nil {
		return err
	}
	// Launch learning script.
	log.Printf("Job %d on port %d launch!\n", j.TaskId, port)
	log.Println("python", j.LaunchFile, "-p", strconv.Itoa(port))
	cmd := exec.Command("python", j.LaunchFile, "-p", strconv.Itoa(port))
	cmd.Start()
	cmd.Wait()
	log.Printf("Job %d on port %d finished!\n", j.TaskId, port)
	return nil
}

type JobResult struct {
	Success     bool
	Description error
	onJob       Job
}

// Worker starts a learning script
func (l *Launcher) worker(parent context.Context, id int, jobs <-chan Job, results chan<- JobResult) {
	log.Printf("Worker %d started\n", id)
	ctx, cancel := context.WithCancel(parent)
	defer cancel()
	port := 11311 + id
	e := Environment{}
	e.Init(port, id)

	onJobStart := make(chan struct{})
	onJobFinish := make(chan struct{})
	wg := sync.WaitGroup{}
	wg.Add(1)

	go e.Serve(ctx, onJobStart, onJobFinish, &wg)

L:
	for {
		select {
		case _ = <-ctx.Done():
			log.Printf("DBG Worker %d cancel signal\n", id)
			break L
		case job := <-jobs:
			log.Printf("Worker %d job %d assigned\n", id, job.TaskId)
			onJobStart <- struct{}{}
			log.Printf("30 second prevent waiting! // 2 for tests", id, job.TaskId)
			time.Sleep(10 * time.Second)
			success := true
			log.Printf("Env %d job %d .Do started", id, job.TaskId)
			err := job.Do(port)
			log.Printf("Env %d job %d .Do FINISHED", id, job.TaskId)
			if err != nil {
				success = false
				log.Printf("Worker %d job %d finished with an error\n", id, job.TaskId)
			}
			log.Printf("Worker %d job %d finished without errors\n", id, job.TaskId)
			results <- JobResult{
				Success:     success,
				Description: err,
				onJob:       job,
			}
			onJobFinish <- struct{}{}
		}
	}
	wg.Wait()
}

func (l *Launcher) checkPool() (bool, int) {
	for k, v := range l.ActivePool {
		if v == nil {
			return true, k
		}
	}
	return false, -1
}

func (l *Launcher) Start() {
	log.Println("Starting Launcher...")
	// Create a pool of workers.
	jobs := make(chan Job, l.PoolSize)
	results := make(chan JobResult, l.PoolSize)
	cancelFunctions := map[int]*context.CancelFunc{}
	for id := 0; id < int(l.PoolSize); id++ {
		ctx, cancel := context.WithCancel(context.Background())
		cancelFunctions[id] = &cancel
		go l.worker(ctx, id, jobs, results)
	}
	// trigger
	taskId := 0
	for {
		select {
		case _ = <-l.NeedToCheck:
			log.Print("Checking waiting queue")
			anyEmpty, workerId := l.checkPool()
			if len(l.WaitQueue) > 0 && anyEmpty {
				log.Print("Found a new task to start!")
				job := Job{
					Config:     l.WaitQueue[0],
					LaunchFile: l.WaitLaunchFiles[0],
					WorkerId:   workerId,
					TaskId:     taskId,
				}
				l.WaitQueue, _ = deleteMapStrInt(l.WaitQueue, 0)
				l.WaitLaunchFiles, _ = deleteStrings(l.WaitLaunchFiles, 0)
				// to pool
				l.ActivePool[workerId] = &job
				jobs <- job
				go func() { l.NeedToCheck <- true }()
			}
		case jobResult := <-results:
			if !jobResult.Success {
				log.Printf("Task %d failed and returned to queue\n", jobResult.onJob.TaskId)
				l.WaitQueue = append(l.WaitQueue, jobResult.onJob.Config)
				l.WaitLaunchFiles = append(l.WaitLaunchFiles, jobResult.onJob.LaunchFile)
			}
			log.Printf("Task %d success\n", jobResult.onJob.TaskId)
			log.Printf("Worker %d is free\n", jobResult.onJob.WorkerId)
			l.ActivePool[jobResult.onJob.WorkerId] = nil
			go func() { l.NeedToCheck <- true }()

		case id := <-l.RestartContainer:
			// stop container simulation with its learning script
			log.Println("DBG Cancel invocation")
			(*cancelFunctions[id])()
			// wait a while until OS free resources
			time.Sleep(time.Second * 30)
			// Free ActivePool
			l.ActivePool[id] = nil
			// restart container
			log.Println("DBG New container start")
			ctx, cancel := context.WithCancel(context.Background())
			cancelFunctions[id] = &cancel
			go l.worker(ctx, id, jobs, results)
			// wait untill the env. launched
			time.Sleep(time.Second * 10)
			// Check wait queue
			log.Println("DBG Check")
			go func() { l.NeedToCheck <- true }()

		case <-l.Done:
			for id, cancel := range cancelFunctions {
				log.Printf("Cancel context %d\n", id)
				(*cancel)()
			}
		}
		taskId++
	}
}

type ResponseGetConfigs struct {
	Configs         []map[string]interface{} `json:"configs"`
	WaitLaunchFiles []string                 `json:"launch_files"`
	Msg             string                   `json:"msg"`
}

// List configs in scripts/learning_scripts
func (l *Launcher) GetConfigs(pat string) (ResponseGetConfigs, error) {
	if pat == "" {
		pat = "template"
	}
	resp := ResponseGetConfigs{}
	var templates []string
	// list dir scripts/learning_scripts
	dir, err := filepath.Abs(filepath.Dir(os.Args[0]))
	common.FailOnError(err)
	dir, _ = filepath.Split(dir)
	dir = dir[:len(dir)-1]
	dir = filepath.Join(dir, "scripts", "learning_scripts")
	// keep only templates
	files, err := os.ReadDir(dir)
	common.FailOnError(err)
	for _, dirEntry := range files {
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
	Queue       []map[string]interface{} `json:"queue"`
	LaunchFiles []string                 `json:"launch_files"`
	// PoolSize 	uint8						`json:"pool_size"`
	Msg string `json:"msg"`
}

// return wait queue
func (l *Launcher) GetQueue() (ResponseGetQueue, error) {
	resp := ResponseGetQueue{}
	resp.Queue = append(resp.Queue, l.WaitQueue...)
	resp.LaunchFiles = append(resp.LaunchFiles, l.WaitLaunchFiles...)
	// resp.PoolSize = l.PoolSize
	return resp, nil
}

type ResponseGetPool struct {
	Pool        []map[string]interface{} `json:"pool"`
	LaunchFiles []string                 `json:"launch_files"`
	Msg         string                   `json:"msg"`
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
	Config     map[string]interface{} `json:"config"`
	LaunchFile string                 `json:"launch_file"`
	Msg        string                 `json:"msg"`
}

type ResponseCreateTask struct {
	Msg string `json:"msg"`
}

// Add a task to the config
func (l *Launcher) CreateTask(reqRaw io.ReadCloser) (ResponseCreateTask, error) {
	req := RequestCreateTask{}
	body, err := ioutil.ReadAll(reqRaw)
	if err != nil {
		return ResponseCreateTask{}, err
	}
	err = json.Unmarshal(body, &req)
	if err != nil {
		return ResponseCreateTask{}, err
	}
	l.WaitQueue = append(l.WaitQueue, req.Config)
	l.WaitLaunchFiles = append(l.WaitLaunchFiles, req.LaunchFile)
	l.NeedToCheck <- true
	l.db.SaveConfig(req.Config)
	return ResponseCreateTask{"Created with success!"}, nil
}

type RequestUpdateTask struct {
	Config map[string]interface{} `json:"config"`
	TaskId int                    `json:"task_id"`
}

type ResponseUpdateTask struct {
	Found bool   `json:"found"`
	Msg   string `json:"msg"`
}

// update task config in wait queue
func (l *Launcher) UpdateTask(reqRaw io.ReadCloser) (ResponseUpdateTask, error) {
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
		resp.Msg = "taskId is higher than queue length"
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
	l.db.SaveConfig(req.Config)
	return resp, nil
}

type RequestDeleteTask struct {
	TaskId int `json:"task_id"`
}

type ResponseDeleteTask struct {
	Found bool   `json:"found"`
	Msg   string `json:"msg"`
}

func (l *Launcher) DeleteActiveTask(reqRaw io.ReadCloser) (r ResponseDeleteTask, e error) {
	defer func() {
		if r := recover(); r != nil {
			r = ResponseDeleteTask{
				Found: false,
				Msg:   "fatal error",
			}
			e = errors.New("something went wrong within Launcher.DeleteActiveTask")
		}
	}()

	req := RequestDeleteTask{}
	resp := ResponseDeleteTask{}
	body, err := ioutil.ReadAll(reqRaw)
	common.FailOnError(err)
	err = json.Unmarshal(body, &req)
	common.FailOnError(err)

	id := req.TaskId

	log.Println("HERE")

	if id < len(l.ActivePool) {
		l.RestartContainer <- id
		resp.Found = true
	} else {
		resp.Found = false
	}

	return resp, nil
}

func (l *Launcher) DeleteTask(reqRaw io.ReadCloser) (ResponseDeleteTask, error) {
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
		resp.Msg = "taskId is higher than queue length"
		return resp, nil
	}

	l.db.DeleteConfig(l.WaitQueue[taskId]["experiment_series"].(string), l.WaitQueue[taskId]["experiment"].(string))
	var result bool
	l.WaitQueue, result = deleteMapStrInt(l.WaitQueue, taskId)
	if result == false {
		resp.Found = false
		resp.Msg = "could not delete from config pool"
		return resp, nil
	}

	l.WaitLaunchFiles, result = deleteStrings(l.WaitLaunchFiles, taskId)
	if result == false {
		resp.Found = false
		resp.Msg = "could not delete from launch files"
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
func deleteMapStrInt(a []map[string]interface{}, i int) (m []map[string]interface{}, f bool) {
	defer func() {
		if r := recover(); r != nil {
			m = nil
			f = false
		}
	}()
	var aNew []map[string]interface{}
	aNew = a[:i]
	aNew = append(aNew, a[i+1:]...)
	return aNew, true
}

// custom delete from string array by index function
func deleteStrings(a []string, i int) ([]string, bool) {
	defer func() {
		if r := recover(); r != nil {
		}
	}()
	var aNew []string
	aNew = a[:i]
	aNew = append(aNew, a[i+1:]...)
	return aNew, true
}

func (l *Launcher) Close() {
	l.Done <- struct{}{}
}

package main

import (
	"context"
	"log"
	"os"
	"os/exec"
	"strconv"
	"sync"
	"syscall"
	"time"
)

// Environment is a sole learning environment structure.
type Environment struct {
	port     int
	active   bool
	occupied bool
	addr     string
}

// Initialize the environment state and pass the port information.
func (e *Environment) Init(port int) {
	e.port = port
	e.active = false
	e.occupied = false
	e.addr = "http://localhost:" + strconv.Itoa(port)
}

// Start is a gouroutine to start the environment.
// Destruction is invkoed through CancelFunc of the context package.
func (e *Environment) start(ctx context.Context, wg *sync.WaitGroup) {
	os.Setenv("ROS_MASTER_URI", e.addr)
	// Start a process:
	log.Println("Starting on", e.port)
	cmd := exec.Command("roslaunch", "-p", strconv.Itoa(e.port), "test", "test.launch")
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	cmd.Start()
	log.Println("e.start before select")
	select {
	case _ = <-ctx.Done():
		log.Println("e.start done")
		// kill the process on cancel
		pgid, err := syscall.Getpgid(cmd.Process.Pid)
		if err == nil {
			// NOte that minus sign is for the groupd pid.
			syscall.Kill(-pgid, 15)
		}
		cmd.Wait()
		log.Println("e.start Wait-Done")
		wg.Done()
	}
}

// Serve method manages one environment creation/killing.
// On new task arrival the environment is started if it was not active.
// As soon as the task finishes, a timer is launched for 15 minutes.
// In case of new tasks absence for the latter time period, the envrionment is closed.
// Then, it waits for new tasks.
func (e *Environment) Serve(parent context.Context, onJobArrival, onJobFinish chan struct{}, parentWG *sync.WaitGroup) {
	var (
		wg     sync.WaitGroup
		ctx    context.Context
		cancel context.CancelFunc
		timer  *time.Timer
	)
	os.Setenv("ROS_MASTER_URI", e.addr)
	timer = time.NewTimer(time.Minute * 15)
	loop:
	for {
		select {
		// Launch roslaunch and indicate the environment as active on new task.
		case _ = <-onJobArrival:
			log.Println("e.Serve new job")
			if timer != nil {
				timer.Stop()
				log.Println("e.Serve new job stopping timer")
			}
			if !e.active {
				log.Println("e.Serve new job not active")
				ctx, cancel = context.WithCancel(parent)
				wg.Add(1)
				go e.start(ctx, &wg)
				e.active = true
				e.occupied = true
			} else {
				log.Println("e.Serve new job occupation true")
				e.occupied = true
			}
		// Start a timer until firing or new task arrival.
		case _ = <-onJobFinish:
			log.Println("e.Serve onJobFinish")
			timer = time.NewTimer(time.Second * 2)
			e.occupied = false
		// Kill environment after a period of time.
		// It indicates that there are no new tasks yet.
		case _ = <-timer.C:
			if e.active && !e.occupied {
				log.Println("e.Serve on timer C")
				cancel()
				wg.Wait()
				e.active = false
			}
		// Kill environment and exit on parent context cancelation.
		case _ = <-parent.Done():
			// The method cancel logic is realised in the start method.
			// cancel()
			wg.Wait()
			log.Println("e.Serve on cancel")
			e.occupied = false
			e.active = false
			break loop
		}
	}
	parentWG.Done()
	log.Println("e.Serve on very end")
}


type Job struct {
	Config 		map[string]interface{}		`json:"config"`
	WorkerId	string						`json:"worker_id"`
	LaunchFile	string						`json:"launch_file"`
	Id 			int							`json:"id"`
}

// Launch a learning script
func (j *Job) Do () error {

	return nil
}

type JobResult struct {
	Success 		bool
	Description 	error
	JobId 			int
}

// start python script
func  worker(parent context.Context, id int, jobs <-chan Job, results chan<- JobResult) {
	ctx, _ := context.WithCancel(parent)

	port := 11311+id
	e := Environment{}
	e.Init(port)

	onJobStart := make(chan struct{})
	onJobFinish := make(chan struct{})
	wg := sync.WaitGroup{}
	wg.Add(1)
	go e.Serve(ctx, onJobStart, onJobFinish, &wg)

	L:
	for {
		select {
		case _ = <-ctx.Done():
			break L
		case job := <-jobs:
			onJobStart <- struct{}{}
			time.Sleep(30*time.Second)
			success := true
			err := job.Do()
			if err!=nil {
				log.Println("Job was not finished properly.")
				success = false
			}
			results <- JobResult{
				Success:     success,
				Description: err,
				JobId:       job.Id,
			}
			onJobFinish <- struct{}{}
		}
	}
	wg.Wait()
}


func main() {
		e := Environment{}
		wg := sync.WaitGroup{}
		e.Init(11312)
		newTask := make(chan struct{})
		onJobFinish := make(chan struct{})
		ctx, cancel := context.WithCancel(context.Background())
		wg.Add(1)
		go e.Serve(ctx, newTask, onJobFinish, &wg)
		newTask <- struct{}{}
		log.Println(1)
		time.Sleep(4 * time.Second)
		onJobFinish <- struct{}{}
		log.Println(2)
		time.Sleep(3 * time.Second)
		newTask <- struct{}{}
		log.Println(3)
		time.Sleep(4 * time.Second)
		onJobFinish <- struct{}{}
		cancel()
		wg.Wait()
}

package master

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

type Environment struct {
	port     int
	active   bool
	occupied bool
	addr     string
	id		int
}

// Initialize the environment state and pass the port information.
func (e *Environment) Init(port, id int) {
	e.port = port
	e.active = false
	e.occupied = false
	e.addr = "http://localhost:" + strconv.Itoa(port)
	e.id = id
}

// Start is a gouroutine to start the environment.
// Destruction is invkoed through CancelFunc of the context package.
func (e *Environment) start(ctx context.Context, wg *sync.WaitGroup) {
	os.Setenv("ROS_MASTER_URI", e.addr)
	// Start a process:
	log.Printf("Environment started with id %d on port %d.\n", e.id, e.port)
	// cmd := exec.Command("roslaunch", "-p", strconv.Itoa(e.port), "test", "test.launch")
	// cmd := exec.Command("roslaunch", "-p", strconv.Itoa(e.port), "backend", "learning.launch")
	sport := strconv.Itoa(e.port)
	cmd := exec.Command("roslaunch", "-p", sport, "backend", "learning.launch", "port:="+sport)

	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	err := cmd.Start()
	if err!=nil {
		panic("Critical panic env start")
	}
	select {
	case _ = <-ctx.Done():
		log.Printf("Signal to stop the simulation environment. %d\n", e.id)
		// kill the process on cancel
		pgid, err := syscall.Getpgid(cmd.Process.Pid)
		if err == nil {
			// NOte that minus sign is for the groupd pid.
			syscall.Kill(-pgid, 15)
		}
		err = cmd.Wait()
		if err != nil{
			log.Println("Do cmd wait did not succeed")
		}
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
	log.Printf("Contaienr %d started.\n", e.id)
	// Wait for 2 minutes without tasks; 1 for tests.
	timer = time.NewTimer(time.Minute * 2)
	loop:
	for {
		select {
		// Launch roslaunch and indicate the environment as active on new task.
		case _ = <-onJobArrival:
			log.Printf("Contaienr %d received job.\n", e.id)
			if timer != nil {
				timer.Stop()
				log.Printf("Contaienr %d stopped timer.\n", e.id)
			}
			if !e.active {
				log.Printf("Contaienr %d was not active, launcinf simulation.\n", e.id)
				ctx, cancel = context.WithCancel(parent)
				wg.Add(1)
				go e.start(ctx, &wg)
				e.active = true
				e.occupied = true
			} else {
				log.Printf("Contaienr %d was active, indicated itself as occupied.\n", e.id)
				e.occupied = true
			}
		// Start a timer until firing or new task arrival.
		case _ = <-onJobFinish:
			log.Printf("Contaienr %d, timer started for 15 minutes.\n", e.id)
			timer = time.NewTimer(time.Minute * 15)
			e.occupied = false
		// Kill environment after a period of time.
		// It indicates that there are no new tasks yet.
		case _ = <-timer.C:
			if e.active && !e.occupied {
				log.Printf("Contaienr %d, timer fired.\n", e.id)
				cancel()
				wg.Wait()
				e.active = false
			}
		// Kill environment and exit on parent context cancelation.
		case _ = <-parent.Done():
			// The method cancel logic is realised in the start method.
			// cancel()
			wg.Wait()
			log.Printf("Contaienr %d, cancel signal received.\n", e.id)
			e.occupied = false
			e.active = false
			break loop
		}
	}
	parentWG.Done()
}


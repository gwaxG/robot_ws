package remote_app

import (
	"encoding/json"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
	"log"
)

type TaskServiceReq struct {
	Task string
}

type TaskServiceRes struct {
	Success bool
	Err     string
}

type TaskService struct {
	msg.Package `ros:"learning"`
	TaskServiceReq
	TaskServiceRes
}

type ConfServiceReq struct {
}

type ConfServiceRes struct {
	Conf string
}

type ConfService struct {
	msg.Package `ros:"learning"`
	ConfServiceReq
	ConfServiceRes
}

type rosClient struct {
	n              *goroslib.Node
	serviceGetConf *goroslib.ServiceClient
	serviceAddTask *goroslib.ServiceClient
	task 			map[string]interface{}
}

func (t *rosClient) close() {
	t.n.Close()
	t.serviceGetConf.Close()
	t.serviceAddTask.Close()
}

func (t *rosClient) init() {
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_sc",
		MasterAddress: "127.0.0.1:11311",
	})
	t.n = n
	FailedOnError(err, "ros node creation")

	get, err := goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: n,
		Name: "get_conf",
		Srv:  &ConfService{},
	})
	t.serviceGetConf = get
	FailedOnError(err, "service creation get conf")

	add, err := goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: n,
		Name: "add_task",
		Srv:  &TaskService{},
	})
	t.serviceAddTask = add
	FailedOnError(err, "service creation add task")
}

func (t *rosClient) addTask() {
	jsonSer, err := json.Marshal(t.task)
	FailedOnError(err, "can not get the task")
	taskReq := TaskServiceReq{string(jsonSer)}
	taskRes := TaskServiceRes{}
	err = t.serviceAddTask.Call(&taskReq, &taskRes)
	FailedOnError(err, "can not call the add_task service")
	log.Println("Task added!")
}

func (t *rosClient) getConf() {
	confReq := ConfServiceReq{}
	confRes := ConfServiceRes{}
	err := t.serviceGetConf.Call(&confReq, &confRes)
	FailedOnError(err, "can not get the task configuration")
	err = json.Unmarshal([]byte(confRes.Conf), &t.task)
	FailedOnError(err, "unable to unmarshal provided task configuration")
	log.Println("Conf updated!", t.task)
}
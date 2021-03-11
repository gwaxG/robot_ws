package master

import (
	"path"
	"runtime"
)

type Worker struct {
	Config 		map[string]interface{} 	`json:"config"`
	Occupied 	bool					`json:"occupied"`
}

type Launcher struct {
	WaitPool 		[]Worker 	`json:"wait_pool"`
	ActivePool 		[]Worker 	`json:"active_pool"`
	PoolSize		uint8		`json:"pool_size"`
}

func (l *Launcher) AddToWaitPool(){

}

func (l *Launcher) DeleteFromWaitPool(){

}

func (l *Launcher) DeleteFromActivePool(){

}

func getConfigs() []map[string]interface{} {
	// path := getConfigPath()
	
	// return path.Join(basepath, "scripts/learning_scripts")
	return []map[string]interface{}{}
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

func (l *Launcher) ReadConfigs(){

}

func (l *Launcher) Init(poolSize uint8){
	l.PoolSize = poolSize
	l.ActivePool = make([]Worker, poolSize)
	l.WaitPool = []Worker{}
}

type ResponseListConfigs struct {

	Msg		string `json:"msg"`
}

func (l *Launcher) ListConfigs() (ResponseListConfigs, error){
	resp := ResponseListConfigs{}
	return resp, nil
}

type ResponseCrudTask struct {

}

func (l *Launcher) CrudTask(config, action string) (ResponseCrudTask, error){
	resp := ResponseCrudTask{}
	return resp, nil
}

type ResponseListQueue struct {

}

func (l *Launcher) ListQueue() (ResponseListQueue, error) {
	resp := ResponseListQueue{}
	return resp, nil
}

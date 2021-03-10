package master

type Launcher struct {}

func (l *Launcher) Init(){}

func (l *Launcher) Launch(){}

type ResponseListConfigs struct {

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

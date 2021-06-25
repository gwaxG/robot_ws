package master

import "testing"

var l *Launcher

func Init(){
	l = &Launcher{
		WaitQueue:       []map[string]interface{}{},
		WaitLaunchFiles: []string{},
		ActivePool:      map[int]*Job{},
		PoolSize:        3,
		NeedToCheck:     make(chan bool),
	}
	go l.Start()
}

func TestCreateTask(t *testing.T) {}

func TestReadTask(t *testing.T) {
	got := 0.0
	want := 40.0
	if got != want {
		t.Errorf("got %.2f want %.2f", got, want)
	}
}

func TestUpdateTask(t *testing.T) {
	got := 0.0
	want := 40.0
	if got != want {
		t.Errorf("got %.2f want %.2f", got, want)
	}
}

func TestDeleteTask(t *testing.T) {
	got := 0.0
	want := 40.0
	if got != want {
		t.Errorf("got %.2f want %.2f", got, want)
	}
}



func TestExample (t *testing.T) {
	got := 0.0
	want := 40.0
	if got != want {
		t.Errorf("got %.2f want %.2f", got, want)
	}
}


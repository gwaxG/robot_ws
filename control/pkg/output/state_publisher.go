package output

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

type StatePublisher struct {
	receiver chan state.State
    pub 	 *goroslib.Publisher
}

func (p *StatePublisher) Init(msgChan chan state.State) {
	var err error
	p.receiver = msgChan
	p.pub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  connections.RosConnection(),
		Topic: "robot/state",
		Msg:   &state.State{},
	})
	utils.FailOnError(err, "can not create publisher state")
}

func (p *StatePublisher) Publish() {
	msg := state.State{}
	for {
		msg = <- p.receiver
		p.pub.Write(&msg)
	}
}

type FakeStatePublisher struct {
	receiver chan state.State
}

func (p *FakeStatePublisher) Init(msgChan chan state.State) {
	p.receiver = msgChan
}

func (p *FakeStatePublisher) Publish() {
	for {
		_ = <- p.receiver
	}
}
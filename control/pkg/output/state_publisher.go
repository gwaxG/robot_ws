package output

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

type StatePublisher struct {
	receiver chan state.State
	pub *goroslib.Publisher
}

func (p *StatePublisher) Init(n *goroslib.Node, msgChan chan state.State) {
	p.receiver = msgChan
	pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "robot/state",
		Msg:   &state.State{},
	})

	p.pub = pub
	utils.FailOnError(err, "can not create publisher state")

}

func (p *StatePublisher) Publish() {
	msg := state.State{}
	for {
		msg = <- p.receiver
		p.pub.Write(&msg)
	}
}
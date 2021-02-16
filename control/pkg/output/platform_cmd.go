package output

import (
	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
)

type PlatformCmd struct {

}

func (p *PlatformCmd) Init(test bool, actions chan state.State) {
	if test {
		_ = connections.ConnectHostPort("localhost", "10001")
		_ = connections.ConnectHostPort("localhost", "10002")
	} else if !test {
		_ = connections.ConnectHostPort("192.168.0.60", "10001")
		_ = connections.ConnectHostPort("192.168.0.63", "10001")
	}
}

func (p *PlatformCmd) Serve() {

}
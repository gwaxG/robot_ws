package master

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"os"
)

type Ros struct {
	node 			*goroslib.Node
	addAnalytics 	*goroslib.Subscriber
	analyticsCh		chan structs.RolloutAnalytics
}

func (r *Ros) Init(analyticsCh chan structs.RolloutAnalytics) {
	r.analyticsCh = analyticsCh
	var err error
	r.node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "master",
		MasterAddress: os.Getenv("ROS_MASTER_URI"),
	})
	common.FailOnError(err)
}


func (r *Ros) Close() {
	r.node.Close()
}
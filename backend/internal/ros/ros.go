package ros

import (
	"encoding/json"
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/gwaxG/robot_ws/backend/internal/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
)

type Ros struct {
	node 			*goroslib.Node
	addAnalytics 	*goroslib.Subscriber
	analyticsCh		chan structs.Analytics
}

func (r *Ros) Init(analyticsCh chan structs.Analytics) {
	r.analyticsCh = analyticsCh
	var err error
	r.node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "backend",
		MasterAddress: "127.0.0.1:11311",
	})
	common.FailOnError(err)
	r.addAnalytics, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "/analytics/rollout",
		Callback: r.onAddingAnalytics,
	})
	common.FailOnError(err)
}

func (r *Ros) onAddingAnalytics(msg *std_msgs.String) {
	var analytics structs.Analytics
	common.FailOnError(json.Unmarshal([]byte(msg.Data), &analytics))
	fmt.Println("Received rollout analytics", analytics)
	r.analyticsCh <- analytics
}

func (r *Ros) onRolloutReturn() {}

func (r *Ros) Close() {
	r.node.Close()

}
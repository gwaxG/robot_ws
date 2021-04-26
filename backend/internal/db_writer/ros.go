package db_writer

import (
	"os"

	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
)

type Ros struct {
	node         *goroslib.Node
	addAnalytics *goroslib.Subscriber
	analyticsCh  chan common.RolloutAnalytics
}

func (r *Ros) Init(analyticsCh chan common.RolloutAnalytics) {
	r.analyticsCh = analyticsCh
	var err error
	r.node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "db_writer",
		MasterAddress: os.Getenv("ROS_MASTER_URI"),
	})
	common.FailOnError(err)
	r.addAnalytics, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "/rollout/analytics",
		Callback: r.onAddingAnalytics,
	})
	common.FailOnError(err)
}

func (r *Ros) onAddingAnalytics(analytics *common.RolloutAnalytics) {
	if analytics.Experiment != "" && analytics.ExpSeries != "" {
		r.analyticsCh <- *analytics
	}
}

func (r *Ros) onRolloutReturn() {}

func (r *Ros) Close() {
	r.node.Close()
}

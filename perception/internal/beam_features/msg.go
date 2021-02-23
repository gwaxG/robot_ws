package beam_features

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"time"
)

type BeamMsg struct {
	msg.Package `ros:"perception"`
	std_msgs.Header
	Width int
	Height int
	Horizontal []float64
	Vertical []float64
}

type RosProxy struct {
	conn 		*goroslib.Node
	featurePub 	*goroslib.Publisher
	imgSub 		*goroslib.Subscriber
	seq 		uint32
}

func (r * RosProxy) Init(subs func(sensor_msgs.Image)) {
	var err error
	r.seq = 1
	r.conn, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "features",
		MasterAddress: "127.0.0.1:11311",
	})
	utils.FailOnError(err, "no roscore")
	r.featurePub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  r.conn,
		Topic: "test_topic",
		Msg:   &BeamMsg{},
	})
	utils.FailOnError(err, "no roscore")
	r.imgSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.conn,
		Topic:    "test_topic",
		Callback: subs,
	})
	utils.FailOnError(err, "no roscore")
}

func (r *RosProxy) Publish(H []float64, V []float64, frame string) {
	r.featurePub.Write(&BeamMsg{
		Header:     std_msgs.Header{
			Seq:     r.seq,
			Stamp:   time.Time{},
			FrameId: frame,
		},
		Width:      len(H),
		Height:     len(V),
		Horizontal: H,
		Vertical:   V,
	})
	r.seq++
}

func (r *RosProxy) Close() {
	r.featurePub.Close()
	r.conn.Close()
}
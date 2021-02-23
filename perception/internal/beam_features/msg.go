package beam_features

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"log"
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
	r.seq = 1
	r.conn, _ = goroslib.NewNode(goroslib.NodeConf{
		Name:          "features",
		MasterAddress: "127.0.0.1:11311",
	})
	depthImageTopic, err := r.conn.ParamGetString("depth_image_topic")
	if err != nil {
		log.Fatal("Can not initialize depth image topic")
	}
	r.featurePub, _ = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  r.conn,
		Topic: "features",
		Msg:   &BeamMsg{},
	})
	r.imgSub, _ = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.conn,
		Topic:    depthImageTopic,
		Callback: subs,
	})
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
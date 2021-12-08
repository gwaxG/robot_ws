package beam_features

import (
	"log"
	"os"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type BeamMsg struct {
	msg.Package `ros:"perception"`
	Header      std_msgs.Header
	Horizontal  std_msgs.Float32MultiArray
	Vertical    std_msgs.Float32MultiArray
}

type RosProxy struct {
	conn            *goroslib.Node
	featurePub      *goroslib.Publisher
	imagePub        *goroslib.Publisher
	imgSub          *goroslib.Subscriber
	seq             uint32
	ImageNoiseLevel int
	GrainSize       int
	Encoding        string
	DistViewMin     float32
	DistViewMax     float32
	Mode            string
}

func (r *RosProxy) SetHandler(subs func(*sensor_msgs.Image)) {
	depthImageTopic, err := r.conn.ParamGetString("depth_image_topic")
	FailOnError(err, "Can not get param string")

	r.imgSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.conn,
		Topic:    depthImageTopic,
		Callback: subs,
	})
	FailOnError(err, "Can not create subscriber")
}

func (r *RosProxy) Init() {
	var err error
	r.seq = 1
	r.conn, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "features",
		MasterAddress: os.Getenv("ROS_MASTER_URI"), // 127.0.0.1:11311
	})
	FailOnError(err, "Can not create node")

	noising, err := r.conn.ParamGetInt("image_noise_level")
	FailOnError(err, "Can not get param string image_noise_level")
	r.ImageNoiseLevel = noising

	encoding, err := r.conn.ParamGetString("depth_image_encoding")
	FailOnError(err, "Can not get param string depth_image_encoding")
	r.Encoding = encoding

	mode, err := r.conn.ParamGetString("perception_mode")
	FailOnError(err, "Can not get param string perception_mode")
	r.Mode = mode

	// Check real camera distance view

	var value int
	value, err = r.conn.ParamGetInt("camera_distance_view_min")
	FailOnError(err, "Can not get param string camera_distance_view_min")
	r.DistViewMin = float32(value) / float32(100)

	value, err = r.conn.ParamGetInt("camera_distance_view_max")
	FailOnError(err, "Can not get param string camera_distance_view_max")
	r.DistViewMax = float32(value) / float32(100)

	graine, err := r.conn.ParamGetInt("grain_size")
	FailOnError(err, "Can not get param string grain_size")
	r.GrainSize = graine

	r.featurePub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  r.conn,
		Topic: "features",
		Msg:   &BeamMsg{},
	})
	FailOnError(err, "Can not create publisher")

	r.imagePub, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  r.conn,
		Topic: "noised_image",
		Msg:   &sensor_msgs.Image{},
	})

	FailOnError(err, "Can not create publisher")
}

func (r *RosProxy) PublishImage(img *sensor_msgs.Image) {
	r.imagePub.Write(img)
}

func (r *RosProxy) Publish(Height []float32, Width []float32, frame string) {
	r.featurePub.Write(&BeamMsg{
		Header: std_msgs.Header{
			Seq:     r.seq,
			Stamp:   time.Time{},
			FrameId: frame,
		},
		Horizontal: std_msgs.Float32MultiArray{
			Data: Width,
		},
		Vertical: std_msgs.Float32MultiArray{
			Data: Height,
		},
	})
	r.seq++
}

func (r *RosProxy) Close() {
	r.featurePub.Close()
	r.conn.Close()
}

func FailOnError(err error, msg string) {
	if err != nil {
		log.Println(msg)
		panic(err)
	}
}

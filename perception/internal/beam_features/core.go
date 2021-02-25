package beam_features

import (
	"encoding/binary"
	"fmt"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"log"
	"math"
)

type Core struct {
	ros RosProxy
}

func (c *Core) Init() {
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
}

func (c *Core) Start () {
	log.Println("Started")
	select {}
}



func (c *Core) Handle(img *sensor_msgs.Image) {
	depthImage := de
	h := img.Height
	w := img.Width
	step := img.Step
	l := uint32(len(img.Data))
	bytes := []byte{}
	for i:=0;i<4;i++{
		bytes = append(bytes, img.Data[614400 + i])
	}
	bits := binary.LittleEndian.Uint32(bytes)
	float := math.Float32frombits(bits)
	fmt.Println(l)
	fmt.Printf("float %f %T\n ", float, float)
	// log.Println("is Big endian", img.IsBigendian)
	/*log.Println(float64(l)/float64(step))
	log.Println(step/w)
	log.Println(img.Encoding)*/
	
	_ = w
	_ = step
	_ = l
	_ = h
}
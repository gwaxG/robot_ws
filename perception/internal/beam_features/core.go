package beam_features

import (
	"log"
	"math"
	"math/rand"
	"sync"

	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/gwaxG/robot_ws/perception/pkg/bridge"
)

const BandWidth = 10

type Core struct {
	ros           RosProxy
	HeightFeatNum int
	WidthFeatNum  int
	BandWidth     int
}

func (c *Core) Init() {
	var err error
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
	c.HeightFeatNum, err = c.ros.conn.ParamGetInt("feature_height")
	FailOnError(err, "can not get param")
	c.WidthFeatNum, err = c.ros.conn.ParamGetInt("feature_width")
	FailOnError(err, "can not get param")
	c.BandWidth, err = c.ros.conn.ParamGetInt("band_width")
	FailOnError(err, "can not get param")
}

func (c *Core) Start() {
	log.Println("Started")
	select {}
}

func calcMean(values []float32) float32 {
	if len(values) == 0 {
		return 0.0
	}
	num := 0
	var sum float32
	for _, val := range values {
		num += 1
		sum += val
	}
	return sum / float32(num)
}

func (c *Core) handleSlice(isHeight bool, img *bridge.Image, dst *[]float32, wg *sync.WaitGroup) {
	defer wg.Done()
	var (
		step   float32
		index  uint32 = 1
		center uint32
		beam   = []float32{}
	)
	if isHeight {
		step = float32(img.Height) / float32(c.HeightFeatNum)
		center = img.Width / 2
		for sliceN := 0; uint32(sliceN) < img.Height; sliceN++ {
			for pixelN := center - uint32(c.BandWidth/2); pixelN < center+uint32(c.BandWidth/2); pixelN++ {
				if !math.IsNaN(float64(img.Rows[sliceN][pixelN])) {
					beam = append(beam, img.Rows[sliceN][pixelN])
				}
			}
			if float32(sliceN) > step*float32(index) {
				*dst = append(*dst, calcMean(beam))
				beam = nil
				index++
			}
		}
	} else {
		step = float32(img.Width) / float32(c.WidthFeatNum)
		center = img.Height / 2
		for sliceN := 0; uint32(sliceN) < img.Width; sliceN++ {
			for pixelN := center - uint32(c.BandWidth/2); pixelN < center+uint32(c.BandWidth/2); pixelN++ {
				if !math.IsNaN(float64(img.Rows[pixelN][sliceN])) {
					beam = append(beam, img.Rows[pixelN][sliceN])
				}
			}
			if float32(sliceN) > step*float32(index) {
				*dst = append(*dst, calcMean(beam))
				beam = nil
				index++
			}
		}
	}
	*dst = append(*dst, calcMean(beam))
}

func (c *Core) Handle(img *sensor_msgs.Image) {
	beamsHeight := []float32{}
	beamsWidth := []float32{}

	decoded := bridge.Decode(img, "depth")
	noised := c.Noise(decoded)
	var wg sync.WaitGroup
	wg.Add(2)
	go c.handleSlice(true, noised, &beamsHeight, &wg)
	go c.handleSlice(false, noised, &beamsWidth, &wg)
	wg.Wait()

	c.ros.Publish(beamsHeight, beamsWidth, img.Header.FrameId)
}

func (c *Core) Noise(dec *bridge.Image) *bridge.Image {
	img := *dec
	// Noising
	grainNumber := int(float64(dec.Height*dec.Width/uint32(c.ros.GrainSize)) * c.ros.ImageNoiseLevel)
	for g := 0; g < grainNumber; g++ {
		h := rand.Intn(int(dec.Height) - c.ros.GrainSize)
		w := rand.Intn(int(dec.Width) - c.ros.GrainSize)
		for hi := 0; hi < c.ros.GrainSize; hi++ {
			for wi := 0; wi < c.ros.GrainSize; wi++ {
				img.Rows[h+hi][w+wi] = 0
			}
		}
	}
	return &img
}

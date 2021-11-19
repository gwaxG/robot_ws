package beam_features

import (
	"log"
	"math"
	"math/rand"
	"sync"

	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/gwaxG/robot_ws/perception/pkg/bridge"
	"gonum.org/v1/gonum/stat/distuv"
)

const BandWidth = 10

type Core struct {
	ros           RosProxy
	HeightFeatNum int
	WidthFeatNum  int
	BandWidth     int
	cnt           int
	mutex         sync.Mutex
	viewDistance  float64
}

func (c *Core) Init() {
	c.mutex = sync.Mutex{}
	var err error
	c.ros = RosProxy{}
	c.ros.Init(c.Handle)
	c.HeightFeatNum, err = c.ros.conn.ParamGetInt("feature_height")
	FailOnError(err, "can not get param")
	c.WidthFeatNum, err = c.ros.conn.ParamGetInt("feature_width")
	FailOnError(err, "can not get param")
	c.BandWidth, err = c.ros.conn.ParamGetInt("band_width")
	FailOnError(err, "can not get param")
	var dist int
	dist, err = c.ros.conn.ParamGetInt("view_distance")
	FailOnError(err, "can not get param")
	c.viewDistance = float64(dist) / 100.
	FailOnError(err, "can not parse param")
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
	distanceCut := c.CutDistance(decoded)
	noised := c.Noise(distanceCut)
	var wg sync.WaitGroup
	wg.Add(2)
	go c.handleSlice(true, noised, &beamsHeight, &wg)
	go c.handleSlice(false, noised, &beamsWidth, &wg)
	wg.Wait()
	c.ros.Publish(beamsHeight, beamsWidth, img.Header.FrameId)
}

type SourceUint64 struct {
}

func (c *Core) Noise(dec *bridge.Image) *bridge.Image {
	img := *dec
	normal := distuv.Normal{0, 1., nil}
	// Noising
	grainNumber := int(float64(dec.Height) * float64(dec.Width) / float64(c.ros.GrainSize) * float64(c.ros.ImageNoiseLevel) / 100.)
	gsize := c.ros.GrainSize

	UsedCPU := 8
	cover := grainNumber / 4
	start := 0
	end := cover
	var wg sync.WaitGroup

	for i := 0; i < UsedCPU; i++ {
		wg.Add(1)
		go c.AddNoise(&wg, start, end, gsize, &normal, dec, &img)
		start += cover
		end += cover
	}

	// Uncomment to see noised images.
	c.ros.PublishImage(bridge.Encode(&img))

	return &img
}

func (c *Core) CutDistance(dec *bridge.Image) *bridge.Image {
	img := *dec
	for h := 0; h < int(img.Height); h++ {
		for w := 0; w < int(img.Width); w++ {
			if img.Rows[h][w] < float32(c.viewDistance) {
				img.Rows[h][w] = 0.0
			}
		}
	}
	return &img
}

func (c *Core) AddNoise(wg *sync.WaitGroup, start, end, gsize int, normal *distuv.Normal, dec, img *bridge.Image) {
	for g := start; g < end; g++ {
		randomF := math.Abs(normal.Rand())
		size := int(randomF*float64(gsize)) % gsize
		if size == 0 {
			size = 1
		}

		h := rand.Intn(int(dec.Height) - size)
		w := rand.Intn(int(dec.Width) - size)
		for hi := 0; hi < size; hi++ {
			for wi := 0; wi < size; wi++ {
				img.Rows[h+hi][w+wi] = 0
			}
		}
	}
	wg.Done()
}

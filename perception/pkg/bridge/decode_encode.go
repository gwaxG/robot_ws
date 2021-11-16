package bridge

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"

	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
)

type row interface {
	addRow(*[]float32)
}

type depthRow struct {
	values *[]float32
}

func (d *depthRow) addRow(array *[]float32) {
	d.values = array
}

type Image struct {
	// seen that Pixels can be different, we precise the image type: depth, rgb, rgbd, rgba
	Type string
	// number of columns
	Width uint32
	// number of rows
	Height uint32
	// General description
	// Encoding 	string
	// IsBigEndian bool
	// ChanNum 	uint8
	// Rows, numeration starts from the upper left corner
	Rows [][]float32
}

var rosImage *sensor_msgs.Image

func appendPixel(pixel *[]byte, Row *[]float32) {
	bits := binary.LittleEndian.Uint32(*pixel)
	float := math.Float32frombits(bits)
	*Row = append(*Row, float)
	*pixel = nil
}

func Decode(img *sensor_msgs.Image, t string) *Image {
	rosImage = img
	var proc = Image{
		t,
		img.Width,
		img.Height,
		[][]float32{},
	}
	pixel := []byte{}
	Row := []float32{}
	for i := 0; i < len(img.Data)/4; i++ {
		pixel = []byte{img.Data[i*4], img.Data[i*4+1], img.Data[i*4+2], img.Data[i*4+3]}
		appendPixel(&pixel, &Row)
		if len(Row) == int(img.Width) {
			proc.Rows = append(proc.Rows, Row)
			Row = nil
		}
	}
	return &proc
}

func Float32ToByte(f float32) []byte {
	var buf bytes.Buffer
	err := binary.Write(&buf, binary.LittleEndian, f)

	if err != nil {
		fmt.Println("binary.Write failed:", err)
	}
	return buf.Bytes()
}

func Encode(img *Image) *sensor_msgs.Image {
	rosImage.Data = make([]uint8, 0)
	for i := 0; i < int(img.Height); i++ {
		for j := 0; j < int(img.Width); j++ {
			pixel := img.Rows[i][j]
			bytes := Float32ToByte(pixel)
			rosImage.Data = append(rosImage.Data, bytes...)
		}
	}
	return rosImage
}

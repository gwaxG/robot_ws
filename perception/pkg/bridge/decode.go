package bridge

import (
	"encoding/binary"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"math"
)

type row interface {
	addRow(*[]float32)
}

type depthRow struct {
	values *[]float32
}

func (d* depthRow) addRow(array *[]float32){
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
	Rows []interface{}
}

func (i *Image) appendRow(row interface{}) {
	switch row.(type){
	case float32:
		i.Rows = append(i.Rows, row)
	default:
		panic("strange type")
	}
}

func Decode(img *sensor_msgs.Image, t string) *Image {
	var proc = Image{
		t,
		img.Width,
		img.Height,
		[]interface{},
	}
	pixel := []byte{}
	Row := []float32{}
	for i, byt := range img.Data {
		if i > 0 && i % 4 == 0 {
			bits := binary.LittleEndian.Uint32(pixel)
			float := math.Float32frombits(bits)
			Row = append(Row, float)
			pixel = nil
			if len(Row) == int(img.Width) {
				proc.appendRow(Row)
				Row= nil
			}
		}

		pixel = append(pixel, byt)
	}
	bytes := []byte{}
	for i:=0;i<4;i++{
		bytes = append(bytes, img.Data[614400 + i])
	}
	return &proc
}

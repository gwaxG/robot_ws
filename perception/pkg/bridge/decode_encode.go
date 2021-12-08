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

func appendPixel(pixel *[]byte, Row *[]float32, isBigEndian uint8) {
	/*var bits uint32
	if isBigEndian == 0 {
		bits = binary.LittleEndian.Uint32(*pixel)
	} else {
		bits = binary.BigEndian.Uint32(*pixel)
	}*/
	bits := binary.LittleEndian.Uint32(*pixel)
	float := math.Float32frombits(bits)
	
	*Row = append(*Row, float)
	*pixel = nil
}

func appendPixelUint16(pixel *[]byte, Row *[]float32, DistViewMin, DistViewMax float32, isBigEndian uint8) {
	// Get uint16 value.
	var value uint16
	if isBigEndian == 0 {
		value = binary.LittleEndian.Uint16(*pixel)
	} else {
		value = binary.BigEndian.Uint16(*pixel)
	}

	// uint16 value corresponds to milimeters	
	distanceValue := float32(value) / 1000.0	

	*Row = append(*Row, distanceValue)
	*pixel = nil
}

// Method to decode the simulation depth image with encoding 32FC1.
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
		appendPixel(&pixel, &Row, img.IsBigendian)
		if len(Row) == int(img.Width) {
			proc.Rows = append(proc.Rows, Row)
			Row = nil
		}
	}
	return &proc
}

// Ad-hoc method to decode the real-world depth image issued from the RealSense SR300 camera with encoding 16UC1.
func DecodeRealSense(img *sensor_msgs.Image, t string, DistViewMin, DistViewMax float32, endian uint8) *Image {
	rosImage = img
	var proc = Image{
		t,
		img.Width,
		img.Height,
		[][]float32{},
	}

	pixel := []byte{}

	Row := []float32{}
	for i := 0; i < len(img.Data)/2; i++ {
		pixel = []byte{img.Data[i*2], img.Data[i*2+1]}
		appendPixelUint16(&pixel, &Row, DistViewMin, DistViewMax, endian)
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

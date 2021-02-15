// ros and net connection utilities
package connections

import (
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"net"
	"time"
)

// Ping ip:port and return coonection if the former is available.
func ConnectHostPort(host, port string) net.Conn {
	addr := host + ":" + port
	timeout := 100 * time.Millisecond
	conn, err := net.DialTimeout("tcp", addr, timeout)
	if err != nil {
		return nil
	} else {
		return conn
	}
}

// Ping roscore and return ROS node if the former is available.
func ConnectRos(simTime bool) *goroslib.Node {
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "jag_control",
		MasterAddress: "127.0.0.1:11311",
	})
	utils.FailOnError(err, "Failed to connect to ROS master")
	return n
}

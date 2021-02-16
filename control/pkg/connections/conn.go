// ros and net connection utilities
package connections

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"net"
	"sync"
	"time"
)

var lock = &sync.Mutex{}

var _RobotConn map[string]net.Conn
// Ping ip:port and return coonection if the former is available.
func ConnectHostPort(host, port string) net.Conn {
	lock.Lock()
	defer lock.Unlock()

	var conn net.Conn
	var err error
	var ok bool
	if _RobotConn == nil {
		_RobotConn = make(map[string]net.Conn)
	}
	addr := host + ":" + port
	if conn, ok = _RobotConn[addr]; !ok {
		conn, err = net.DialTimeout("tcp", addr, 100 * time.Millisecond)
		utils.FailOnError(err, "robot connection problem")
		_RobotConn[addr] = conn
	}
	return conn
}

var _RosConn *goroslib.Node
// Return ROS node
func RosConnection() *goroslib.Node {
	lock.Lock()
	defer lock.Unlock()
	if _RosConn == nil {
		n, err := goroslib.NewNode(goroslib.NodeConf{
			Name:          "jag_control",
			MasterAddress: "127.0.0.1:11311",
		})
		_RosConn = n
		utils.FailOnError(err, "Failed to connect to ROS master")
	}
	return _RosConn
}

func Close() {
	var err error
	if _RosConn != nil {
		err = _RosConn.Close()
		utils.FailOnError(err, "Can not close ROS connection")
	}
	if _RobotConn != nil {
		for k, v := range _RobotConn {
			err = v.Close()
			utils.FailOnError(err, fmt.Sprintf("issue with %s robot connection close", k))
		}
	}
}

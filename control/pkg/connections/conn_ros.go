// ros and net connection utilities
package connections

import (
	"os"

	"github.com/aler9/goroslib"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
)

var _RosConn *goroslib.Node

// Return ROS node
func RosConnection() *goroslib.Node {
	lock.Lock()
	defer lock.Unlock()
	if _RosConn == nil {
		n, err := goroslib.NewNode(goroslib.NodeConf{
			Name:          "jag_control",
			MasterAddress: os.Getenv("ROS_MASTER_URI"),
		})
		_RosConn = n
		utils.FailOnError(err, "Failed to connect to ROS master")
	}
	return _RosConn
}

// ros and net connection utilities
package connections

import (
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
			MasterAddress: "127.0.0.1:11311",
		})
		_RosConn = n
		utils.FailOnError(err, "Failed to connect to ROS master")
	}
	return _RosConn
}
package connections

import (
	"fmt"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"sync"
)

var lock = &sync.Mutex{}

func Close() {
	var err error
	if _RosConn != nil {
		err = _RosConn.Close()
		utils.FailOnError(err, "Can not close ROS connection")
	}
	if _RobotConn != nil {
		err = _RobotConn.baseConn.Close()
		utils.FailOnError(err, fmt.Sprintf("issue with robot connection closing"))
		err = _RobotConn.armConn.Close()
		utils.FailOnError(err, fmt.Sprintf("issue with robot connection closing"))
	}
}


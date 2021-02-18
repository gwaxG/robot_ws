// ros and net connection utilities
package connections

import (
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"net"
	"time"
)

var _RobotConn *_PlatformConn

type _PlatformConn struct {
	baseConn net.Conn
	armConn net.Conn
}

func (pc *_PlatformConn) ConnectHostPort(addrBase, addrArm string) {
	var conn net.Conn
	var err error
	lock.Lock()
	defer lock.Unlock()
	conn, err = net.DialTimeout("tcp", addrArm, 100 * time.Millisecond)
	utils.FailOnError(err, "arm connection problem")
	pc.armConn = conn
	conn, err = net.DialTimeout("tcp", addrBase, 100 * time.Millisecond)
	utils.FailOnError(err, "robot connection problem")
	pc.baseConn = conn
}

func Init(addrBase, addrArm string) {
	_RobotConn = &_PlatformConn{}
	_RobotConn.ConnectHostPort(addrBase, addrArm)
}

func WriteToBase(cmd string) {
	lock.Lock()
	defer lock.Unlock()
	_, err := _RobotConn.baseConn.Write([]byte(cmd))
	utils.FailOnError(err, "Error in writing to the base board")
}

func WriteToArm(cmd string) {
	lock.Lock()
	defer lock.Unlock()
	_, err := _RobotConn.armConn.Write([]byte(cmd))
	utils.FailOnError(err, "Error in writing to the arm board")
}

func GetConnBase() *net.Conn {
	return &_RobotConn.baseConn
}

func GetConnArm() *net.Conn {
	return &_RobotConn.armConn
}


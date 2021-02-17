package input

import (
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"os"
	"os/exec"
	"unicode/utf8"
)

type Keyboard struct {
	stateChange 	chan state.State
	done 			chan bool
	reset 			chan bool
	keyboardUsage	chan bool

}

func (k *Keyboard) Init (stateChange chan state.State, reset, done, keyboardUsage chan bool) {
	k.stateChange = stateChange
	k.done = done
	k.reset = reset
	k.keyboardUsage = keyboardUsage
	exec.Command("stty", "-F", "/dev/tty", "cbreak", "min", "1").Run()
	// do not display entered characters on the screen
	// exec.Command("stty", "-F", "/dev/tty", "-echo").Run()
}

func (k *Keyboard) Serve () {
	var reset, done, setVelocity bool
	b := make([]byte, 1)
	st := &state.State{}
	for {
		os.Stdin.Read(b)
		reset, done, setVelocity = k.handleKeyPress(b, st)
		if reset {
			k.reset <- true
		} else if done {
			k.done <- true
		} else {
			if setVelocity {
				go func() { k.keyboardUsage <- true }()
			}
			k.stateChange <- *st
		}
		state.Reset(st)
	}
}

func  (k *Keyboard) handleKeyPress(b []byte, st *state.State) (bool, bool, bool) {
	key, _ := utf8.DecodeRune(b)
	var reset, done, setVelocity bool
	switch key {
	case 'z':
		st.Linear = 0.1
		setVelocity = true
	case 'q':
		st.Angular = -0.1
		setVelocity = true
	case 's':
		st.Linear = -0.1
		setVelocity = true
	case 'd':
		st.Angular = 0.1
		setVelocity = true
	case 'r':
		st.FrontFlippers = 0.1
	case 'f':
		st.FrontFlippers = -0.1
	case 't':
		st.RearFlippers = 0.1
	case 'g':
		st.RearFlippers = -0.1
	case 'y':
		st.ArmJoint1 = 0.1
	case 'h':
		st.ArmJoint1 = -0.1
	case 'u':
		st.ArmJoint2 = 0.1
	case 'j':
		st.ArmJoint2 = -0.1
	case 'a':
		reset = true
	case 'p':
		done = true
	}
	return reset, done, setVelocity
}

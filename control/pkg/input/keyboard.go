package input

import (
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"os"
	"unicode/utf8"
)

type Keyboard struct {
	stateChange chan state.State
	done chan bool
}

func (k *Keyboard) Init (stateChange chan state.State, done chan bool) {
	k.stateChange = stateChange
	k.done = done
	// exec.Command("stty", "-F", "/dev/tty", "cbreak", "min", "1").Run()
	// do not display entered characters on the screen
	// exec.Command("stty", "-F", "/dev/tty", "-echo").Run()
}

func (k *Keyboard) Serve () {
	b := make([]byte, 1)
	st := &state.State{}
	for {
		os.Stdin.Read(b)
		k.handleKeyPress(b, st)
		k.stateChange <- *st
		state.Reset(st)
	}
}

func  (k *Keyboard) handleKeyPress(b []byte, st *state.State){
	key, _ := utf8.DecodeRune(b)
	switch key {
	case 'z':
		st.Linear = 0.1
	case 'q':
		st.Angular = -0.1
	case 's':
		st.Linear = -0.1
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
	case 'p':
		k.done <- true
	}
}

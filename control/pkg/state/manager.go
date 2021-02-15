package state

import (
	"reflect"
)

type Manager struct {
	state State
	limits Limits
	names []string
}

func (m *Manager) Init(){
	m.state = State{}
	m.limits = Limits{}

	state := reflect.ValueOf(State{})
	for i:=0; i<state.NumField(); i++ {
		if i != 0 { // ignoring msg.Package message field
			m.names = append(m.names, state.Type().Field(i).Name)
		}
	}
}
// Control actions correctness through velocity and joint limits approval
func (m *Manager) Monitor(change State) (State, State) {
	actions := State{}

	changes := reflect.ValueOf(change)
	var min, max, value, changeValue, last float64
	for _, name := range m.names {
		changeValue = changes.FieldByName(name).Float()
		value = reflect.ValueOf(m.state).FieldByName(name).Float()
		min = reflect.ValueOf(m.limits).FieldByName(name).FieldByName("Min").Float()
		max = reflect.ValueOf(m.limits).FieldByName(name).FieldByName("Max").Float()
		if name == "Linear" || name == "Angular" {
			value = 0.0
		}
		if value + changeValue < min {
			last = min
			value = min
			changeValue = min - last
		} else if value + changeValue > max {
			last = max
			value = max
			changeValue = max - last
		} else {
			value = value + changeValue
		}

		// correct action vector
		reflect.Indirect(reflect.ValueOf(&actions)).FieldByName(name).SetFloat(changeValue)
		// produce state vector
		reflect.Indirect(reflect.ValueOf(&m.state)).FieldByName(name).SetFloat(value)
	}
	return m.state, actions
}

/*
type C struct {
	A float64
}
func main() {
	c := C{1.0}
	cc := C{0.1}
	names := []string{"A"}
	changes := reflect.ValueOf(cc)
	var change_value float64
	for _, name := range names {
		change_value = changes.FieldByName(name).Float()
		reflect.Indirect(reflect.ValueOf(&c)).FieldByName(name).SetFloat(reflect.ValueOf(c).FieldByName(name).Float()+change_value)
	}
	fmt.Println(c)
}
*/

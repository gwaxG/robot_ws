package state

import (
	"encoding/json"
	"github.com/gwaxG/robot_ws/control/pkg/utils"
	"io/ioutil"
	"log"
	"os"
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
	m.limits.Init()

	state := reflect.ValueOf(State{})
	for i:=0; i<state.NumField(); i++ {
		if i != 0 { // ignoring msg.Package state field
			m.names = append(m.names, state.Type().Field(i).Name)
		}
	}
}
// Control actions correctness through velocity and joint limits approval
func (m *Manager) Monitor(set bool, change State) (State, State) {
	actions := State{}

	changes := reflect.ValueOf(change)
	var min, max, value, changeValue, last float64
	for _, name := range m.names {
		changeValue = changes.FieldByName(name).Float()
		value = reflect.ValueOf(m.state).FieldByName(name).Float()
		min = reflect.ValueOf(m.limits).FieldByName(name).FieldByName("Min").Float()
		max = reflect.ValueOf(m.limits).FieldByName(name).FieldByName("Max").Float()
		if name == "Linear" || name == "Angular"{
			log.Println("set", set, changeValue)
			if set && changeValue != 0.0{
				value = 0.0
			}
		}
		if value + changeValue < min {
			last = value
			value = min
			changeValue = min - last
		} else if value + changeValue > max {
			last = value
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

func (m *Manager) Reset() (State, State) {
	actions := State{}
	var currentFieldValue float64
	for _, name := range m.names {
		currentFieldValue = reflect.ValueOf(m.state).FieldByName(name).Float()
		reflect.Indirect(reflect.ValueOf(&actions)).FieldByName(name).SetFloat(-currentFieldValue)
		reflect.Indirect(reflect.ValueOf(&m.state)).FieldByName(name).SetFloat(0.0)
	}
	return m.state, actions
}

func (m *Manager) Save() {
	jsonString, _ := json.Marshal(&saveState{
		FrontFlippers: m.state.FrontFlippers,
		RearFlippers:  m.state.RearFlippers,
		ArmJoint1:     m.state.ArmJoint1,
		ArmJoint2:     m.state.ArmJoint2,
		ArmJoint3:     m.state.ArmJoint3,
		ArmJoint4:     m.state.ArmJoint4,
	})
	ioutil.WriteFile("platform_state.json", jsonString, os.ModePerm)
}

func (m *Manager) Load() (State, State) {
	jsonFile, err := os.Open("platform_state.json")
	defer jsonFile.Close()
	utils.FailOnError(err, "can not open platform_state.json")
	byteValue, _ := ioutil.ReadAll(jsonFile)
	var state saveState
	_ = json.Unmarshal(byteValue, &state)
	change := State{
		FrontFlippers: -state.FrontFlippers,
		RearFlippers:  -state.RearFlippers,
		ArmJoint1:     -state.ArmJoint1,
		ArmJoint2:     -state.ArmJoint2,
		ArmJoint3:     -state.ArmJoint3,
		ArmJoint4:     -state.ArmJoint4,
	}
	return m.state, change
}
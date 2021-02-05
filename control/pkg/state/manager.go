package state

type Manager struct {
	stateChange chan State
}

func (m *Manager) Init(stateChange chan State){
	m.stateChange = stateChange
}

func (m *Manager) Monitor(change *State){

}


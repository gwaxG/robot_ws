package state

type PlatformState struct {
	Allowed bool
	leftMotorCounts int64
	rightMotorCounts int64
	temp float64
	current float64
}

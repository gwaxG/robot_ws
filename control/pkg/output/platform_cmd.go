package output

import (
	"log"
	"strconv"
	"time"

	"github.com/gwaxG/robot_ws/control/pkg/connections"
	"github.com/gwaxG/robot_ws/control/pkg/state"
)

type PlatformCmd struct {
	stateActionCh chan []state.State
	stopReleaseCh chan string
}

const PI = 3.14
const Latency = 25
const MAXVel = 400
const AngFlipperRes = 5700.0 / (2 * PI) // 7600
const AngArmRes = 5700.0 / (2 * PI)

func (p *PlatformCmd) Init(stopReleaseCh chan string, test bool, StateChange chan []state.State) {
	p.stateActionCh = StateChange
	p.stopReleaseCh = stopReleaseCh
	if test {
		connections.Init("localhost:10001", "localhost:10002")
	} else {
		connections.Init("192.168.0.60:10001", "192.168.0.63:10001")
	}
	go p.ping()
}

func (p *PlatformCmd) Serve() {
	var StateChange []state.State
	var SetState, Change state.State
	go func() {
		switch <-p.stopReleaseCh {
		case "stop":
			p.stopMotors()
		case "release":
			p.releaseMotors()
		}
	}()
	for {
		StateChange = <-p.stateActionCh

		Change = StateChange[1]
		SetState = StateChange[0]
		p.serveBase(&SetState, &Change)
		p.serveFlipper(&SetState, &Change)
		p.serveArm(&SetState, &Change)
	}
}

func (p *PlatformCmd) serveBase(setState, change *state.State) {
	// Base width, wheel radius
	D, R := 0.6, 0.085
	left := (setState.Linear + setState.Angular*D/2) / R / (2 * PI) * MAXVel
	right := (setState.Linear - setState.Angular*D/2) / R / (2 * PI) * MAXVel
	cmd := "MMW !M " + strconv.Itoa(int(-left)) + " " + strconv.Itoa(int(right)) + "\r\n"
	connections.WriteToBase(cmd)
}

func (p *PlatformCmd) serveFlipper(setState, change *state.State) {
	var cmd string
	fr := int(change.FrontFlippers * AngFlipperRes)
	rr := int(change.RearFlippers * AngFlipperRes)
	if fr != 0 {
		cmd = "MM2 !PR 1 " + strconv.Itoa(fr) + "\r\n"
		connections.WriteToBase(cmd)
		// time to wait for ack from server
		if rr != 0 {
			time.Sleep(time.Millisecond * Latency)
		}
	}
	if rr != 0 {
		cmd = "MM2 !PR 2 " + strconv.Itoa(rr) + "\r\n"
		connections.WriteToBase(cmd)
	}
}

func (p *PlatformCmd) serveArm(setState, change *state.State) {
	var cmd string
	arm1 := int(change.ArmJoint1 * AngArmRes)
	arm2 := int(change.ArmJoint2 * AngArmRes)
	if arm1 != 0 {
		cmd = "!PR 1 " + strconv.Itoa(-arm1) + "\r"
		connections.WriteToArm(cmd)
		// time to wait for ack from server
		if arm2 != 0 {
			time.Sleep(time.Millisecond * Latency)
		}
	}
	if arm2 != 0 {
		cmd = "!PR 2 " + strconv.Itoa(arm2) + "\r"
		connections.WriteToArm(cmd)
	}
}

// ping platform every 200 ms to avoid disconnection
func (p *PlatformCmd) ping() {
	ticker := time.NewTicker(200 * time.Millisecond)
	cmd := "PING\r\n"
	for {
		select {
		case <-ticker.C:
			connections.WriteToBase(cmd)
		}
	}
}

func (p *PlatformCmd) stopMotors() {
	log.Println("Motors stopped")
	stopBase := "MMW !EX\r\n"
	connections.WriteToBase(stopBase)
	stopFF := "MM2 !EX\r\n"
	connections.WriteToBase(stopFF)
	stopRF := "MM3 !EX\r\n"
	connections.WriteToBase(stopRF)
}

func (p *PlatformCmd) releaseMotors() {
	log.Println("Motors released")
	releaseBase := "MMW !MG\r\n"
	connections.WriteToBase(releaseBase)
	releaseFF := "MM2 !MG\r\n"
	connections.WriteToBase(releaseFF)
	releaseRR := "MM3 !EX\r\n"
	connections.WriteToBase(releaseRR)
}

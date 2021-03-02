package monitor

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"github.com/gwaxG/robot_ws/monitor/pkg/simulation_structs"
	"log"
)

type ROS struct {
	node         	*goroslib.Node
	addToBackend 	*goroslib.ServiceClient
	goalInfo	 	*goroslib.ServiceClient
	stairInfo	 	*goroslib.ServiceClient
	newRollout	 	*goroslib.ServiceProvider
	startNewRollout	*goroslib.ServiceProvider
	stepReturn		*goroslib.ServiceProvider
	odomSub			*goroslib.Subscriber
	robSub			*goroslib.Subscriber
	rolloutState 	*structs.RolloutState
	robotStateCh	chan state.State
	odometryCh		chan nav_msgs.Odometry
	initCh 			chan bool
}

func (r *ROS) Init(state *structs.RolloutState, robotStateCh chan state.State, odometryCh chan nav_msgs.Odometry, initCh chan bool){
	r.robotStateCh	= robotStateCh
	r.odometryCh = odometryCh
	r.rolloutState = state
	r.initCh = initCh
	fmt.Println("ROS init")
	var err error
	r.node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "monitor",
		MasterAddress: "127.0.0.1:11311",
	})
	FailOnError(err)
	/* Communication infrastructure */
	// Send to backend client
	r.addToBackend, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: r.node,
		Name: "analytics/rollout",
		Srv:  &structs.Analytics{},
	})
	FailOnError(err)
	// New rollout service
	r.newRollout, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "rollout/new",
		Srv:      &structs.NewRollout{},
		Callback: r.onNewRollout,
	})
	FailOnError(err)
	// Trigger rollout
	r.startNewRollout, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "rollout/start",
		Srv:      &std_srvs.Trigger{},
		Callback: r.onStartNewRollout,
	})
	FailOnError(err)
	// Step return
	r.stepReturn, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "rollout/step_return",
		Srv:      &structs.StepReturn{},
		Callback: r.onStepReturn,
	})
	FailOnError(err)
	/* Logic subscribers */
	// robot state
	r.robSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "robot/state",
		Callback: r.onRobotState,
	})
	FailOnError(err)
	// robot odometry
	r.odomSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "odometry",
		Callback: r.onOdometry,
	})
	FailOnError(err)
	//
	r.goalInfo, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: r.node,
		Name: "goal_info",
		Srv:  &simulation_structs.GoalInfo{},
	})
	FailOnError(err)
	r.stairInfo, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: r.node,
		Name: "stair_info",
		Srv:  &simulation_structs.StairInfo{},
	})
	FailOnError(err)
}

func (r *ROS) onOdometry(odom *nav_msgs.Odometry) {
	fmt.Println("odometry received")
	r.odometryCh <- *odom
}

func (r *ROS) onRobotState(robotState *state.State) {
	fmt.Println("robot state received")
	r.robotStateCh <- *robotState
}
func onNewRollout(_ *structs.NewRolloutReq) *structs.NewRolloutRes{
	return &structs.NewRolloutRes{Received: true}
}

// Assign new parameters of rollout and put its values to the there
func (r *ROS) onNewRollout(req *structs.NewRolloutReq) *structs.NewRolloutRes{
	r.rolloutState.Experiment = req.Experiment
	r.rolloutState.Seq = req.Seq
	r.rolloutState.Sensors = req.Sensors
	r.rolloutState.Arm = req.Arm
	r.rolloutState.Angular = req.Angular
	r.rolloutState.Progress = 0.
	r.rolloutState.Reward = 0.
	r.rolloutState.StepReward = 0.
	r.rolloutState.CogDeviation = 0.
	r.rolloutState.StepCogDeviation = 0.
	r.rolloutState.CogHeight = 0.
	r.rolloutState.StepCogHeight = 0.
	r.rolloutState.Done = false
	r.rolloutState.Started = false
	r.rolloutState.Closest = 10000.0
	r.rolloutState.MaximumDist = 0.
	r.rolloutState.Published = false

	r.initCh <- true
	return &structs.NewRolloutRes{Received: true}
}

// Reset the rollout state
func (r *ROS) onStartNewRollout(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes{
	log.Println("Start new rollout call")
	r.rolloutState.Started = true
	return &std_srvs.TriggerRes{Success: true, Message: ""}
}

// Handler of the StepReturn service
func (r *ROS) onStepReturn(_ *structs.StepReturnReq) *structs.StepReturnRes{
	log.Println("Step return call")
	msg := &structs.StepReturnRes{
		Reward: r.rolloutState.StepReward,
		Done:   r.rolloutState.Done,
	}
	r.rolloutState.StepReward = 0
	r.rolloutState.StepCogDeviation = 0
	r.rolloutState.StepCogHeight = 0
	return msg
}

// Send to backend the rollout results
func (r *ROS) SendToBackend() {
	fmt.Println("TO BACKEND")
	msg := structs.AnalyticsReq{
		Experiment:   r.rolloutState.Experiment,
		Seq:          r.rolloutState.Seq,
		Sensors:      r.rolloutState.Sensors,
		Arm:          r.rolloutState.Arm,
		Angular:      r.rolloutState.Angular,
		Progress:     r.rolloutState.Progress,
		Reward:       r.rolloutState.Reward,
		CogDeviation: r.rolloutState.CogDeviation,
		CogHeight:    r.rolloutState.CogHeight,
	}
	resp := structs.AnalyticsRes{}
	log.Println("DBG2")

	FailOnError(r.addToBackend.Call(msg, &resp))
	log.Printf("Send to backed %s %d\n", r.rolloutState.Experiment, r.rolloutState.Seq)
}

func (r *ROS) Close() {
	r.addToBackend.Close()
	r.node.Close()
}

func FailOnError(err error) {
	if err != nil {
		log.Fatal(err)
	}
}


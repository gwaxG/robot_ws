package monitor

import (
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	simStructs "github.com/gwaxG/robot_ws/simulation/pkg/structs"
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
}

func (r *ROS) Init(state *structs.RolloutState, robotStateCh chan state.State, odometryCh chan nav_msgs.Odometry){
	r.robotStateCh	= robotStateCh
	r.odometryCh = odometryCh
	r.rolloutState = state

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
		Name: "analytics",
		Srv:  &structs.Analytics{},
	})
	FailOnError(err)
	// New rollout service
	r.newRollout, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "new_rollout",
		Srv:      &structs.NewRollout{},
		Callback: r.onNewRollout,
	})
	FailOnError(err)
	// Trigger rollout
	r.startNewRollout, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "start_rollout",
		Srv:      &std_srvs.Trigger{},
		Callback: r.onStartNewRollout,
	})
	FailOnError(err)
	// Step return
	r.stepReturn, err = goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     r.node,
		Name:     "step_return",
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
		Srv:  &simStructs.GoalInfo{},
	})
	FailOnError(err)
	r.stairInfo, err = goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: r.node,
		Name: "stair_info",
		Srv:  &simStructs.StairInfo{},
	})
	FailOnError(err)
}

func (r *ROS) onOdometry(odom *nav_msgs.Odometry) {
	r.odometryCh <- *odom
}

func (r *ROS) onRobotState(robotState *state.State) {
	r.robotStateCh <- *robotState
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
	r.rolloutState.Published = false
	return &structs.NewRolloutRes{Received: true}
}

// Reset the rollout state
func (r *ROS) onStartNewRollout(_ std_srvs.TriggerReq) *std_srvs.TriggerRes{
	r.rolloutState.Started = true
	return &std_srvs.TriggerRes{Success: true, Message: ""}
}

// Handler of the StepReturn service
func (r *ROS) onStepReturn(_ structs.StepReturnReq) *structs.StepReturnRes{
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


package monitor

import (
	"encoding/json"
	"log"
	"os"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/nav_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
	"github.com/gwaxG/robot_ws/control/pkg/state"
	"github.com/gwaxG/robot_ws/monitor/pkg/simulation_structs"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
)

type ROS struct {
	node            *goroslib.Node
	addToBackend    *goroslib.Publisher
	goalInfo        *goroslib.ServiceClient
	stairInfo       *goroslib.ServiceClient
	newRollout      *goroslib.ServiceProvider
	startNewRollout *goroslib.ServiceProvider
	stepReturn      *goroslib.ServiceProvider
	devSub          *goroslib.Subscriber
	safeSub         *goroslib.Subscriber
	odomSub         *goroslib.Subscriber
	robSub          *goroslib.Subscriber
	rolloutState    *structs.RolloutState
	robotStateCh    chan state.State
	odometryCh      chan nav_msgs.Odometry
	comm            *map[string]interface{}

	expSeries string
}

func (r *ROS) Init(state *structs.RolloutState, comm *map[string]interface{}) {
	r.rolloutState = state
	r.comm = comm

	var err error
	r.node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "monitor",
		MasterAddress: os.Getenv("ROS_MASTER_URI"),
	})
	FailOnError(err)
	/* Communication infrastructure */
	// Send to backend client
	r.addToBackend, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  r.node,
		Topic: "/analytics/rollout",
		Msg:   &std_msgs.String{},
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

	// safety deviation
	r.devSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "safety/relative_deviation",
		Callback: r.onSafetyDeviation,
	})
	// safety angular
	r.safeSub, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     r.node,
		Topic:    "safety/angular",
		Callback: r.onSafetyAngular,
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

func (r *ROS) updateExpSeries() {
	var err error
	r.expSeries, err = r.node.ParamGetString("exp_series_name")
	FailOnError(err)
}

func (r *ROS) onOdometry(odom *nav_msgs.Odometry) {
	(*r.comm)["Odometry"].(chan nav_msgs.Odometry) <- *odom
}

func (r *ROS) onRobotState(robotState *state.State) {
	(*r.comm)["RobotState"].(chan state.State) <- *robotState
}

func (r *ROS) onSafetyDeviation(msg *std_msgs.Float32) {
	(*r.comm)["SafetyDeviation"].(chan float32) <- msg.Data
}

func (r *ROS) onSafetyAngular(msg *std_msgs.Float32) {
	(*r.comm)["SafetyAngular"].(chan float32) <- msg.Data
}

// func on_NewRollout(_ *structs.NewRolloutReq) *structs.NewRolloutRes{
// 	return &structs.NewRolloutRes{Received: true}
// }

// Assign new parameters of rollout and put its values to the there
func (r *ROS) onNewRollout(req *structs.NewRolloutReq) *structs.NewRolloutRes {
	// fmt.Println("New rollout")
	r.updateExpSeries()
	(*r.comm)["NewRollout"].(func(*structs.NewRolloutReq, string))(req, r.expSeries)
	return &structs.NewRolloutRes{Received: true}
}

// Reset the rollout state
func (r *ROS) onStartNewRollout(_ *std_srvs.TriggerReq) *std_srvs.TriggerRes {
	// fmt.Println("start new rollout")
	(*r.comm)["StartNewRollout"].(func())()
	return &std_srvs.TriggerRes{Success: true, Message: ""}
}

// Handler of the StepReturn service
func (r *ROS) onStepReturn(_ *structs.StepReturnReq) *structs.StepReturnRes {
	// fmt.Printf("one step %f %d %v\n", r.rolloutState.StepReward, r.rolloutState.TimeSteps, r.rolloutState.Done)
	flushedStepReturn := (*r.comm)["StepReturn"].(func() float32)()
	msg := &structs.StepReturnRes{
		Reward: flushedStepReturn,
		Done:   r.rolloutState.Done,
	}
	return msg
}

func maxFloat32(arr *[]float32) float32 {
	if len(*arr) == 0 {
		return 0.
	}
	value := (*arr)[0]
	for _, elem := range *arr {
		if value < elem {
			value = elem
		}
	}
	return value
}

func sumFloat32(arr *[]float32) float32 {
	if len(*arr) == 0 {
		return 0.
	}
	var value float32
	for _, elem := range *arr {
		value += elem
	}
	return value
}

func meanFloat32(arr *[]float32) float32 {
	if len(*arr) == 0 {
		return 0.
	}
	var (
		value float32
		n     int
	)
	for _, elem := range *arr {
		value += elem
		n += 1
	}
	return value / float32(n)
}

// Send to backend the rollout results
func (r *ROS) SendToBackend() {
	// log.Println("Sending to backend")
	// log.Println(" R", r.rolloutState.Reward)
	// log.Println(" D", meanFloat32(&r.rolloutState.Deviation))
	msg := structs.RolloutAnalytics{
		ExpSeries:  r.rolloutState.ExpSeries,
		Experiment: r.rolloutState.Experiment,
		Seq:        r.rolloutState.Seq,
		Sensors:    r.rolloutState.Sensors,
		Arm:        r.rolloutState.Arm,
		Angular:    r.rolloutState.Angular,
		Progress:   r.rolloutState.Progress,
		Reward:     r.rolloutState.Reward,
		AngularM:   meanFloat32(&r.rolloutState.AngularM),
		Deviation:  meanFloat32(&r.rolloutState.Deviation),
		Accidents:  r.rolloutState.Accidents,
		TimeSteps:  r.rolloutState.TimeSteps,
	}
	encoded, _ := json.Marshal(msg)
	r.addToBackend.Write(&std_msgs.String{
		Data: string(encoded),
	})
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

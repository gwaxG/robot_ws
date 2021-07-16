#ifndef _JAGUAR_PLUGIN_HH_
#define _JAGUAR_PLUGIN_HH_

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/TwistStamped.h>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class JaguarPlugin : public ModelPlugin
  {

  	event::ConnectionPtr updateConnection;
  	double Target_FLP_FR, Target_FLP_FL, Target_FLP_RR, Target_FLP_RL;
  	double       update_rate_;
    double       update_period_;
    common::Time last_update_time_;
    boost::mutex       lock_;
    ros::Publisher joint_pub; 
    sensor_msgs::JointState joint_state_;
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	private: ros::Subscriber rosSub;
	private: ros::CallbackQueue rosQueue;
	private: boost::thread rosQueueThread;
    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
    private: physics::ModelPtr model;
    private: physics::Joint_V joints;
    private: physics::JointPtr joint;
    private: physics::JointPtr right_front_joint;
    private: physics::JointPtr left_front_joint;
    private: physics::JointPtr right_rear_joint;
    private: physics::JointPtr left_rear_joint;
    private: common::PID pid;
	private: std::string flipperTopicName;


    public: JaguarPlugin() 
    {
    	Target_FLP_FR = Target_FLP_FL = Target_FLP_RR = Target_FLP_RL = M_PI / 4; 	
    }

    public: ~JaguarPlugin()
    {
    	rosQueue.clear();
    	rosQueue.disable();
    	rosQueueThread.join();
    }
    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

	    // Safety check
	    if (_model->GetJointCount() == 0)
	    {
	      std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
	      return;
	    }
	    gzdbg<<"messages"<<std::endl;
	    // Store the model pointer for convenience.
	    this->model = _model;
	    // Get the first joint. We are making an assumption about the model
	    // having one joint that is the rotational joint.
	    
	    // Default to zero velocity
	    // Check that the velocity element exists, then read the value

	    if (_sdf->HasElement("flipperTopicName"))
	      	flipperTopicName = _sdf->Get<std::string>("flipperTopicName");  

	    if (_sdf->HasElement("right_front_joint"))
	    	this->right_front_joint = _model->GetJoint(_sdf->Get<std::string>("right_front_joint"));
	    	this->joints.push_back(this->right_front_joint);	
	    if (_sdf->HasElement("left_front_joint"))
	    	this->left_front_joint = _model->GetJoint(_sdf->Get<std::string>("left_front_joint"));
	    	this->joints.push_back(this->left_front_joint);	
	    if (_sdf->HasElement("right_rear_joint"))
	    	this->right_rear_joint = _model->GetJoint(_sdf->Get<std::string>("right_rear_joint"));
	    	this->joints.push_back(this->right_rear_joint);	
	    if (_sdf->HasElement("left_rear_joint"))
	    	this->left_rear_joint = _model->GetJoint(_sdf->Get<std::string>("left_rear_joint"));
	    	this->joints.push_back(this->left_rear_joint);	

	    if (_sdf->HasElement("updateRate"))
	    	update_rate_ = _sdf->Get<double>("updateRate");
	   	else 
	   		update_rate_ = 0.0;

	      // Create the node
	    this->node = transport::NodePtr(new transport::Node());
	    this->node->Init(this->model->GetWorld()->Name());

	    // Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
		    int argc = 0;
		    char **argv = NULL;
		    ros::init(argc, argv, "gazebo_client",
		        ros::init_options::NoSigintHandler);
		}

		update_period_ = (update_rate_ > 0.0)?(1.0/update_rate_):0.0;
      	last_update_time_ = model->GetWorld()->SimTime();

      	
		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		this->joint_pub = this->rosNode->advertise<sensor_msgs::JointState>("/joint_states", 1);

		// Create a named topic, and subscribe to it.
		ros::SubscribeOptions so =
		    ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
		        flipperTopicName,
		        1,
		        boost::bind(&JaguarPlugin::flipperCallback, this, _1),
		        ros::VoidPtr(), &this->rosQueue);
		this->rosSub = this->rosNode->subscribe(so);

		// Spin up the queue helper thread.
		this->rosQueueThread =
		    boost::thread(boost::bind(&JaguarPlugin::QueueThread, this));

		// Automatic loop event
        this->updateConnection = 
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&JaguarPlugin::OnUpdate, this));
	}

	public: void flipperCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg) 
	{
	    boost::mutex::scoped_lock scoped_lock(lock_);
	    Target_FLP_FR = cmd_msg->twist.linear.x;
	    Target_FLP_FL = cmd_msg->twist.linear.y;
	    Target_FLP_RR = cmd_msg->twist.angular.x;
	    Target_FLP_RL = cmd_msg->twist.angular.y;
	}

	/////////////////////////////////////////////////
    public: void Move_A_Joint(physics::JointPtr _joint, double _target_angle)
    {
      float P = _target_angle - _joint->Position();
      P *= 100;
      // See also [JointController](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html)
      //  Set torque fitting power and direction calculated by each angle.
      //  Seting calculated P as torque is very effective to stop shaking legs!!
      _joint->SetForce(0, P);
      // Set PID parameters
      this->model->GetJointController()->SetPositionPID(_joint->GetScopedName(), 
                                                     common::PID(0.4, 1, 0.005));
      // Set distination angle
      this->model->GetJointController()->SetPositionTarget(_joint->GetScopedName(),
                                                                  _target_angle); 
    //ROS_INFO("TA:%f", _target_angle);
    }

    public: void MoveFlipper(void)
    {
      Move_A_Joint(this->right_front_joint, -Target_FLP_FR);
      Move_A_Joint(this->right_rear_joint, Target_FLP_RR);
      Move_A_Joint(this->left_front_joint, -Target_FLP_FL);
      Move_A_Joint(this->left_rear_joint, Target_FLP_RL);
    }

	/// \brief ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}

	public:
	void OnUpdate(void)
	{
	  common::Time current_time = model->GetWorld()->SimTime();
	  double seconds_since_last_update=(current_time-last_update_time_).Double();
	  if(seconds_since_last_update > update_period_)
	  {
	    MoveFlipper();
	    PublishJointStates();
	    last_update_time_ = current_time;
	  }
	}

	void PublishJointStates() {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( this->joints.size() );
    joint_state_.position.resize ( this->joints.size() );
    joint_state_.velocity.resize ( this->joints.size() );

    for ( int i = 0; i < this->joints.size(); i++ ) {
        physics::JointPtr joint = this->joints[i];
        double velocity = joint->GetVelocity( 0 );
		double position = joint->Position ( 0 );
		joint_state_.name[i] = joint->GetName();
		joint_state_.position[i] = position;
		joint_state_.velocity[i] = velocity;
    }
    this->joint_pub.publish ( joint_state_ );
}
  };
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(JaguarPlugin)
}
#endif

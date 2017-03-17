#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64MultiArray.h"


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: common::PID pid;
    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
    private: transport::SubscriberPtr subSensor;
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    ///
    private: ros::Publisher rosPub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Constructor    
    public: VelodynePlugin() {}

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

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // // Default to zero velocity
      // double velocity = 0;

      // // Check that the velocity element exists, then read the value
      // if (_sdf->HasElement("velocity"))
      //   velocity = _sdf->Get<double>("velocity");

      // // test
      // if (_sdf->HasElement("text"))
      //   std::cerr << _sdf->Get<std::string>("text");

      // // Set the joint's target velocity. This target velocity is just
      // // for demonstration purposes.
      // this->model->GetJointController()->SetVelocityTarget(
      //     this->joint->GetScopedName(), velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->GetName());

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
      std::string topicNameSensor = "~/" + this->model->GetName() + "/velodyne/top/sensor/scan";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsg, this);
      this->subSensor = this->node->Subscribe(topicNameSensor,
         &VelodynePlugin::OnSensorMsg, this);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
            "test",// topicName,
            1,
            boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&VelodynePlugin::QueueThread, this));

      // To publish scanner's data
      this->rosPub = this->rosNode->advertise<std_msgs::Float64MultiArray>("sendscan", 10);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      // TODO: Set other things
      //       Depends on what information you need to pass from ROS to Gazebo
      // this->SetVelocity(_msg->x());
      // this->model->SetLinearVel(math::Vector3 {_msg->x(), _msg->y(), _msg->z()});
      this->model->SetRelativePose(math::Pose {_msg->x(), _msg->y(), _msg->z(), 0, 0, 0});
    }

    private: void OnSensorMsg(ConstLaserScanStampedPtr &_msg)
    {
      std_msgs::Float64MultiArray m;

      std::vector<double> &v = m.data;
      for (size_t i = 0; i < _msg->scan().ranges_size(); ++i) {
        v.push_back(_msg->scan().ranges(i));
      }

      this->rosPub.publish(m);

      // std::cerr<< floor(_msg->scan().ranges(26) * 1000) << std::endl;
      // << ' ' << floor(_msg->scan().intensities(20) * 1000) << std::endl;

      // inline double angle_min() const;
      // inline double angle_max() const;
      // inline double angle_step() const;
      // inline double range_min() const;
      // inline double range_max() const;
      // inline ::google::protobuf::uint32 count() const;
      // inline double vertical_angle_min() const; // has_...()
      // inline double vertical_angle_max() const;
      // inline double vertical_angle_step() const;
      // inline ::google::protobuf::uint32 vertical_count() const;
      // inline int ranges_size() const;
      // inline double ranges(int index) const;
      // inline int intensities_size() const;
      // inline double intensities(int index) const;


      //this->model->SetRelativePose(math::Pose {_msg->x(), _msg->y(), _msg->z(), 0, 0, 0});
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // TODO: You can pass other information to Gazebo like this

      // Set the joint's target velocity.

      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float64MultiArrayConstPtr &_msg)
    {
      // this->SetVelocity(_msg->data);
      const std::vector<double> &v = _msg->data;
      this->model->SetRelativePose(math::Pose {v[0], v[1], v[2], v[3], v[4], v[5]});
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
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif

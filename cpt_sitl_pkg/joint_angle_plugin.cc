#ifndef _JOINT_ANGLE_PLUGIN_HH_
#define _JOINT_ANGLE_PLUGIN_HH_

#include <cstdio>
//#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/advertise_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a Joint Angle sensor.
  class Joint_Angle_Plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: Joint_Angle_Plugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {      
      // Store the model pointer for convenience.
      this->model = _model;
      // Get the third joint. 
      this->joint = _model->GetJoints()[2];
      printf("Model and joint stored\n");
      
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif
      printf("Gazebo node initiated\n");
      
      this->pub = this->node->Advertise<gazebo::msgs::Vector2d>("/"+this->model->GetName()+"/joint_angle");
      //this->pub->WaitForConnection();
      //printf("Gazebo publisher connection achieved\n");
      
        
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      printf("ROS node initialized\n");
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      
      ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<std_msgs::Float32>(
          "/" + this->model->GetName() + "/joint_angle",
          1,
          NULL,
          NULL,
          ros::VoidPtr(), 
          &this->rosQueue);
      this->rosPub = this->rosNode->advertise(ops);
      printf("ROS publisher set up\n");
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&Joint_Angle_Plugin::QueueThread, this)); 
        
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Joint_Angle_Plugin::OnUpdate, this));   
    }
    
    public: void OnUpdate()
    {
      gazebo::msgs::Vector2d msg;
      gazebo::msgs::Set(&msg, ignition::math::Vector2d(this->joint->GetAngle(0).Degree(),0));
      this->pub->Publish(msg);
    }  

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A publisher to a named topic.
    private: transport::PublisherPtr pub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;
    
    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS publisher
    private: ros::Publisher rosPub;

    /// \brief A ROS callback queue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Joint angle sensor.
    public: void OnRosMsg()
    {
      std_msgs::Float32 msg;
      msg.data = (float) this->joint->GetAngle(0).Degree();
      this->rosPub.publish(msg);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        Joint_Angle_Plugin::OnRosMsg();
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(Joint_Angle_Plugin)
}
#endif

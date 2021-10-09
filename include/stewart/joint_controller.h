#ifndef _SDF_JOINT_CONTROLLER_HPP_
#define _SDF_JOINT_CONTROLLER_HPP_

#include <memory>
#include <thread>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo
{
    class SDFJointController : public ModelPlugin
    {
    public:
        SDFJointController(){};

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void setPosition(const std_msgs::Float32MultiArray::ConstPtr& msg);

    private:
        void queueThread();

        physics::ModelPtr m_model;

        ros::CallbackQueue m_ros_queue;
        ros::Subscriber m_ros_sub;

        std::thread m_ros_queue_thread;
        std::unique_ptr<ros::NodeHandle> m_nh;
        std::vector<physics::JointPtr> m_joints;
    };
}

#endif // _SDF_JOINT_CONTROLLER_HPP_
#include "stewart/joint_controller.h"

using namespace gazebo;

void SDFJointController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (_model->GetJointCount() <= 0)
    {
        std::cerr << "Invalid joint count, ROS_SDF plugin not loaded\n";
        return;
    }
    else
        std::cout << "\nThe ROS_SDF plugin is attached to model[" << _model->GetName() << "]\n";

    m_model = _model;
    m_joints = m_model->GetJoints();
    common::PID pid(1000.0, 0.1, 100.0); //placeholder values, need to do an input parameter for this

    for (const auto& joint: m_joints)
        m_model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);

    int argc = 0;
    ros::init(argc, nullptr, "gazebo_client", ros::init_options::NoSigintHandler);
    m_nh.reset(new ros::NodeHandle("gazebo_client"));

    // Subscribe to /model_name/position_cmd topic (you publish to this to set positions)
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
        "/" + m_model->GetName() + "/position_cmd", 
        100, 
        boost::bind(&SDFJointController::setPosition, this, _1),
        ros::VoidPtr(),
        &m_ros_queue);

    m_ros_sub = m_nh->subscribe(so);

    // Set up a handler so we don't block here
    m_ros_queue_thread = std::thread(std::bind(&SDFJointController::queueThread, this));
}

void SDFJointController::setPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    auto joints_it = std::begin(m_joints);
    // for (auto data_it = std::begin(msg->data); data_it != std::end(msg->data); ++data_it)
    for (auto data_it : msg->data)
        m_model->GetJointController()->SetPositionTarget((*joints_it++)->GetScopedName(), data_it);
}

void SDFJointController::queueThread()
{
    static const double timeout = 0.01;
    while (m_nh->ok())
        m_ros_queue.callAvailable(ros::WallDuration(timeout));
}

GZ_REGISTER_MODEL_PLUGIN(SDFJointController)
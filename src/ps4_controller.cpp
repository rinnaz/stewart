#include "stewart/ps4_controller.h"

Controller::Controller(int argc, char **argv)
: l_trigger { 0 }, r_trigger { 0 }
{
    ros::init(argc, argv, "ps4_controller");
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/stewart/platform_twist", 100);
    sub = nh.subscribe("/joy", 100, &Controller::callback, this);
}

void Controller::run()
{
    ros::spin();
}

void Controller::callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    l_trigger = -msg->axes[2];
    r_trigger = -msg->axes[5];        
    

    twist_msg.linear.x = -msg->axes[0]/1.5;
    twist_msg.linear.y = msg->axes[1]/1.5;

    twist_msg.angular.x = -msg->axes[4]/3.0;
    twist_msg.angular.y = -msg->axes[3]/3.0;

    // ROS_INFO("l_trigger %f", l_trigger);
    // ROS_INFO("r_trigger %f", r_trigger);  

    if (r_trigger > 0 && twist_msg.linear.z < 1)
        twist_msg.linear.z += 0.005;
    else if (l_trigger > 0 && twist_msg.linear.z > 0)
        twist_msg.linear.z -= 0.005;

    if (msg->buttons[5] && twist_msg.angular.z < M_PI/3.0)
        twist_msg.angular.z += 0.005;
    else if (msg->buttons[4] && twist_msg.angular.z > -M_PI/3.0)
        twist_msg.angular.z -= 0.005;

    if (msg->buttons[0]){
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
    }

    if (msg->buttons[1]){
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
    }

    if (msg->buttons[2]){
        twist_msg.linear.z = 0.5;
    }

    pub.publish(twist_msg);
}

int main(int argc, char **argv)
{
    Controller controller(argc, argv);
    controller.run();

    return 0;
}
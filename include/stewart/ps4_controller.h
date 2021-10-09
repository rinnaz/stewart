#include <cmath>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifndef M_PIl
#define M_PIl (3.14159265358979323846264338327950288)
#endif

class Controller
{
public:
    Controller(int argc, char **argv);
    void run();

private: 
    void callback(const sensor_msgs::Joy::ConstPtr& msg);

    geometry_msgs::Twist twist_msg;
    double l_trigger, r_trigger;

    ros::Publisher pub;
    ros::Subscriber sub;
};
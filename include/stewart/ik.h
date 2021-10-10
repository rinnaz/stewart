#include <cmath>
#include "eigen3/Eigen/Core"
#include <array>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <tuple>
#include <string>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifndef M_PIl
#define M_PIl (3.14159265358979323846264338327950288)
#endif

class IK
{
public: 
    IK(int argc, char **argv);
    void run();

private: 
    void callback(const geometry_msgs::Twist::ConstPtr& msg);

    std::tuple<double, double> processArgs(char **argv);

    Eigen::Matrix<double, 4, 4> transformation_matrix(const double& x,
                                                      const double& y,
                                                      const double& z,
                                                      const double& r,
                                                      const double& p,
                                                      const double& yaw);

    const double height;
    double piston_length, platform_start_height;
    
    std::array<double, 6> bottom_ball_x;
    std::array<double, 6> bottom_ball_y;
    double bottom_ball_z;

    std::array<double, 6> upper_ball_x;
    std::array<double, 6> upper_ball_y;
    double upper_ball_z;

    Eigen::Matrix<double, 6, 4> base, platform;

    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::Float32MultiArray strut_position_msg;
};
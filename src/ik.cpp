#include "stewart/ik.h"

IK::IK(int argc, char **argv)
:   height { 2.0 }
{
    auto bp_distance { height };
    auto base_radius { 2.0 };
    
    if (argc > 2)
    {
        ROS_INFO("Args are passed");
        try{
            std::tie(bp_distance, base_radius) = processArgs(argv);
        } catch ( ros::Exception &e )
        {
            ROS_ERROR("Error occured: %s ", e.what());
        }
    }
    // Platform configuration is currently hardcoded
    // should be reimplemented through config files
    auto base_height     { 0.25 };
    auto platform_height { 0.1 };
    auto platform_radius { base_radius };
    auto ball_radius     { 0.1 };

    auto ball_dtc { sqrt(pow(base_radius, 2) 
                        - 4*base_radius*ball_radius 
                        + 5*pow(ball_radius, 2)) };

    ROS_INFO("Platform ball_dtc = %f", ball_dtc);

    auto ball_separation {
        sqrt(2*pow(ball_dtc, 2)
        - 2*pow(ball_dtc, 2) * cos(M_PI/3 - 2*asin(ball_radius/(base_radius - 2*ball_radius)))) 
        };

    ROS_INFO("Platform ball_separation = %f", ball_separation);

    auto dtm { sqrt(pow(ball_dtc, 2) - pow(0.5 * ball_separation, 2)) };

    this->piston_length = sqrt(pow(bp_distance - 0.5*base_height - 0.5*platform_height, 2) 
                            + pow(ball_separation, 2));

    this->platform_start_height = 0.5 * base_height + bp_distance;

    ROS_INFO("Platform Z = %f", platform_start_height);
    ROS_INFO("Piston lenght = %f", piston_length);

    
    this->bottom_ball_z = base_height;
    this->upper_ball_z  = -0.5 * platform_height;

    this->bottom_ball_x[0] = ball_dtc * cos(M_PI/2 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[0] = ball_dtc * sin(M_PI/2 + asin(ball_radius/(base_radius - 2*ball_radius)));

    this->bottom_ball_x[1] = ball_dtc * cos(M_PI/2 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[1] = ball_dtc * sin(M_PI/2 - asin(ball_radius/(base_radius - 2*ball_radius)));

    this->bottom_ball_x[2] = ball_dtc * cos(-M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[2] = ball_dtc * sin(-M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));

    this->bottom_ball_x[3] = ball_dtc * cos(-M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[3] = ball_dtc * sin(-M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));

    this->bottom_ball_x[4] = ball_dtc * cos(-5*M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[4] = ball_dtc * sin(-5*M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));

    this->bottom_ball_x[5] = ball_dtc * cos(-5*M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->bottom_ball_y[5] = ball_dtc * sin(-5*M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));
    
    // =================================
    
    this->upper_ball_x[0] = ball_dtc * cos(5*M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[0] = ball_dtc * sin(5*M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));

    this->upper_ball_x[1] = ball_dtc * cos(M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[1] = ball_dtc * sin(M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));

    this->upper_ball_x[2] = ball_dtc * cos(M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[2] = ball_dtc * sin(M_PI/6 - asin(ball_radius/(base_radius - 2*ball_radius)));

    this->upper_ball_x[3] = ball_dtc * cos(3*M_PI/2 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[3] = ball_dtc * sin(3*M_PI/2 + asin(ball_radius/(base_radius - 2*ball_radius)));

    this->upper_ball_x[4] = ball_dtc * cos(3*M_PI/2 - asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[4] = ball_dtc * sin(3*M_PI/2 - asin(ball_radius/(base_radius - 2*ball_radius)));

    this->upper_ball_x[5] = ball_dtc * cos(5*M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));
    this->upper_ball_y[5] = ball_dtc * sin(5*M_PI/6 + asin(ball_radius/(base_radius - 2*ball_radius)));

    // strut ends position relative to base
    base << bottom_ball_x[0], this->bottom_ball_y[0], this->bottom_ball_z, 1,
            bottom_ball_x[1], this->bottom_ball_y[1], this->bottom_ball_z, 1,
            bottom_ball_x[2], this->bottom_ball_y[2], this->bottom_ball_z, 1,
            bottom_ball_x[3], this->bottom_ball_y[3], this->bottom_ball_z, 1,
            bottom_ball_x[4], this->bottom_ball_y[4], this->bottom_ball_z, 1,
            bottom_ball_x[5], this->bottom_ball_y[5], this->bottom_ball_z, 1;
    
    // strut ends position relative to platform
    platform << upper_ball_x[0], this->upper_ball_y[0], this->upper_ball_z, 1,
                upper_ball_x[1], this->upper_ball_y[1], this->upper_ball_z, 1,
                upper_ball_x[2], this->upper_ball_y[2], this->upper_ball_z, 1,
                upper_ball_x[3], this->upper_ball_y[3], this->upper_ball_z, 1,
                upper_ball_x[4], this->upper_ball_y[4], this->upper_ball_z, 1,
                upper_ball_x[5], this->upper_ball_y[5], this->upper_ball_z, 1;
    
    for (auto i { 0 }; i < 6; i++)
    {
        strut_position_msg.data.push_back(0);
    }

    ros::init(argc, argv, "ik");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/position_cmd", 100);
    sub = nh.subscribe("stewart/platform_twist", 100, &IK::callback, this);
}

std::tuple<double, double> IK::processArgs(char **argv)
{
    double height_, radius_;
    height_ = std::stod(argv[1]);
    radius_ = std::stod(argv[2]);
    return {height_, radius_};
}

void IK::run()
{
    ros::spin();
}

void IK::callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    auto x { msg->linear.x };
    auto y { msg->linear.y };
    auto z { msg->linear.z };
    auto roll { msg->angular.x };
    auto pitch { msg->angular.y };
    auto yaw { msg->angular.z };

    // Setting current t-matrix of the upper platform
    auto T = this->transformation_matrix(x, y, z + this->platform_start_height, 
                                                        roll, pitch, yaw);
    
    // For each strut calculate desirable length
    for (size_t i { 0 }; i < 6; i++)
    {
        // Platform pose - base pose 
        Eigen::Matrix<double, 4, 1> length = T*platform.row(i).transpose() - base.row(i).transpose();
        
        // Getting lenght from strut ends position 
        strut_position_msg.data[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)) - this->piston_length;
    }
    pub.publish(strut_position_msg);
}

Eigen::Matrix<double, 4, 4> IK::transformation_matrix(const double& x, 
                                                      const double& y, 
                                                      const double& z, 
                                                      const double& r, 
                                                      const double& p, 
                                                      const double& yaw)
{
    Eigen::Matrix<double, 4, 4> T;
    T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
            sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                    -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                        0,                                         0,                                       0, 1;
    return T;
}

int main(int argc, char **argv)
{
    IK ik(argc, argv);
    ik.run();

    return 0;
}
#include <uas_info.h>
#include <tf/tf.h>
#include <Eigen/Dense>


UASInfo::UASInfo(ros::NodeHandle* nh)
{
    //define subscribers 
    state_sub = nh->subscribe<mavros_msgs::State>
        ("mavros/state", 20, &UASInfo::state_cb, this);

    gps_sub = nh->subscribe<sensor_msgs::NavSatFix>
        ("mavros/global_position/global", 20, &UASInfo::gps_cb, this);

    att_sub = nh->subscribe<nav_msgs::Odometry>
        ("mavros/odometry/in", 20, &UASInfo::att_cb, this);

}

void UASInfo::init_vals()
{

}

void UASInfo::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void UASInfo::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    global_pos[0] = msg->latitude;
    global_pos[1] = msg->longitude;
    global_pos[2] = msg->altitude;
}

void UASInfo::att_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    attitude_q[0] = msg->pose.pose.orientation.x;
    attitude_q[1] = msg->pose.pose.orientation.y;
    attitude_q[2] = msg->pose.pose.orientation.z;
    attitude_q[3] = msg->pose.pose.orientation.w;

    attitude_e =  get_euler(attitude_q);
    
    attitude_deg[0] = rad2deg(attitude_e[0]);
    attitude_deg[1] = rad2deg(attitude_e[1]);
    attitude_deg[2] = rad2deg(attitude_e[2]);
    //tf::Matrix3x3 m(q);
}

float UASInfo::rad2deg(float rad_val)
{
    float deg_val = rad_val * 180/ M_PI;
    return deg_val;
}

Eigen::Vector3d UASInfo::get_euler(Eigen::Vector4d quat_vec)
{
    tf::Quaternion q(
        quat_vec[0],
        quat_vec[1],
        quat_vec[2],
        quat_vec[3]
    );
    tf::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    Eigen::Vector3d rpy_vec;
    m.getRPY(rpy_vec[0], rpy_vec[1], rpy_vec[2]);

    return rpy_vec;

}


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

    att_sub = nh->subscribe<afrl_ros::RollPitchYawIADS>
        ("iads_odom", 20, &UASInfo::att_cb, this);

    vfr_sub = nh->subscribe<afrl_ros::HeadingAirspeedAltitudeIADS>
        ("iads_vfr", 20, &UASInfo::vfr_cb, this);

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

// void UASInfo::att_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     attitude_q[0] = msg->pose.pose.orientation.x;
//     attitude_q[1] = msg->pose.pose.orientation.y;
//     attitude_q[2] = msg->pose.pose.orientation.z;
//     attitude_q[3] = msg->pose.pose.orientation.w;

//     attitude_e =  get_euler(attitude_q);
    
//     attitude_deg[0] = rad2deg(attitude_e[0]);
//     attitude_deg[1] = rad2deg(attitude_e[1]);
//     attitude_deg[2] = rad2deg(attitude_e[2]);
//     //tf::Matrix3x3 m(q);
// }

void UASInfo::att_cb(const afrl_ros::RollPitchYawIADS::ConstPtr& msg)
{
    attitude_deg[0] = msg->roll;
    attitude_deg[1] = msg->pitch;
    attitude_deg[2] = msg->yaw;

    attitude_rate_deg[0] = msg->rollspeed;
    attitude_rate_deg[1] = msg->pitchspeed;
    attitude_rate_deg[2] = msg->yawspeed;

}

void UASInfo::vfr_cb(const afrl_ros::HeadingAirspeedAltitudeIADS::ConstPtr& msg)
{
    vfr[0] = msg-> heading;
    vfr[1] = msg-> airspeed;
    vfr[2] = msg-> altitude;
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


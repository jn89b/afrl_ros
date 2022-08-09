#ifndef NULL_UAS_INFO_H
#define UAS_INFO_H


#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/NavSatFix.h>
#include <afrl_ros/RollPitchYawIADS.h>
#include <afrl_ros/HeadingAirspeedAltitudeIADS.h>

#include <mavros_msgs/State.h>
#include <math.h>  
#include <tf/tf.h>


class UASInfo
{
    private:
        ros::NodeHandle nh;

        ros::Subscriber state_sub, gps_sub, att_sub, vfr_sub;
        mavros_msgs::State current_state;

        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
        //void att_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void att_cb(const afrl_ros::RollPitchYawIADS::ConstPtr& msg);
        void vfr_cb(const afrl_ros::HeadingAirspeedAltitudeIADS::ConstPtr& msg);

        Eigen::Vector3d get_euler(Eigen::Vector4d quat_vec);
        float rad2deg(float rad_val);

    public:

        //Eigen::Vector3d attitude_rate;
        UASInfo(ros::NodeHandle* nh);
        void init_vals();

        Eigen::Vector3d global_pos;
        // Eigen::Vector4d attitude_q; //quat attitudes
        // Eigen::Vector3d attitude_e; // euler attitudes in radians
        Eigen::Vector3d attitude_deg; // euler_attitude in degrees
        Eigen::Vector3d attitude_rate_deg; // euler rates in degrees
        Eigen::Vector3d vfr; //heading, airspeed, alt

};
#endif
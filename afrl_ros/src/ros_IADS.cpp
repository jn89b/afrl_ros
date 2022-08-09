/*Sends message from ROS to IADS*/
#include "ros/ros.h"
#include <IADSInterface.h>
#include <uas_info.h>


// enum
// {
//     IADS__TEST_INPUT
//     , IADS__TEST_INPUT2
//     , IADS__TEST_INPUT3
//     , IADS__TEST_INPUT4
//     , IADS__TEST_INPUT5
//     , IADS__TEST_INPUT6
//     , IADS__TOTAL
// };

enum
{
    IADS__ROLL_RATE, 
    IADS__PITCH_RATE,
    IADS__YAW_RATE, 
    IADS__ROLL, 
    IADS__PITCH, 
    IADS__HEADING,
    IADS__AIRSPEED,
    IADS__ALTITUDE,
    IADS__TOTAL
};


//implement iads protocol
//need to set this ip of local host
IADSInterface iads( "192.168.231.110", 1500, IADS__TOTAL);


int main(int argc, char **argv) 
{
    ros::init(argc,argv,"test_user");
    ros::NodeHandle nh;
    
    ros:: Rate rate(100);

    std::string ip_address; 
    //std::string ip = nh.getParam("iads_broker_ip", ip_address);
    //std::cout<<"ip is " << nh.getParam("iads_broker_ip", ip_address) << std::endl;
    

    //instantiate UASInfo object 
    UASInfo uas_info(&nh);

    while (ros::ok())
    {
        std::cout<<"uas rpy deg is " << uas_info.attitude_deg << std::endl;
        iads.setParameter(IADS__ROLL_RATE, uas_info.attitude_rate_deg[0])
            .setParameter(IADS__PITCH_RATE, uas_info.attitude_rate_deg[1])
            .setParameter(IADS__YAW_RATE, uas_info.attitude_rate_deg[2])
            .setParameter(IADS__ROLL, uas_info.attitude_deg[0])
            .setParameter(IADS__PITCH, uas_info.attitude_deg[1])
            .setParameter(IADS__HEADING, uas_info.vfr[0])
            .setParameter(IADS__AIRSPEED, uas_info.vfr[1])
            .setParameter(IADS__ALTITUDE, uas_info.vfr[2])
        .sendData();
        // std::cout<<"sending message"<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



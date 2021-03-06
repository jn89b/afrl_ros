/*Sends message from ROS to IADS*/
#include "ros/ros.h"
#include <IADSInterface.h>
#include <uas_info.h>


enum
{
    IADS__TEST_INPUT
    , IADS__TEST_INPUT2
    , IADS__TEST_INPUT3
    , IADS__TEST_INPUT4
    , IADS__TEST_INPUT5
    , IADS__TEST_INPUT6
    , IADS__TOTAL
};

//implement iads protocol
IADSInterface iads( "10.3.3.10", 1500, IADS__TOTAL);


int main(int argc, char **argv) 
{
    ros::init(argc,argv,"test_user");
    ros::NodeHandle nh;
    
    ros:: Rate rate(20);

    std::string ip_address; 
    //std::string ip = nh.getParam("iads_broker_ip", ip_address);
    std::cout<<"ip is " << nh.getParam("iads_broker_ip", ip_address) << std::endl;
    

    //instantiate UASInfo object 
    UASInfo uas_info(&nh);

    while (ros::ok())
    {
        std::cout<<"uas rpy deg is " << uas_info.attitude_deg << std::endl;
        iads.setParameter( IADS__TEST_INPUT, uas_info.attitude_deg[0])
        .setParameter( IADS__TEST_INPUT2, uas_info.attitude_deg[1])
        .setParameter( IADS__TEST_INPUT3, uas_info.attitude_deg[2])
        .setParameter( IADS__TEST_INPUT4, uas_info.global_pos[0])
        .setParameter( IADS__TEST_INPUT5, uas_info.global_pos[1])
        .setParameter( IADS__TEST_INPUT6, uas_info.global_pos[2])
        .sendData();
        // std::cout<<"sending message"<<std::endl;
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}



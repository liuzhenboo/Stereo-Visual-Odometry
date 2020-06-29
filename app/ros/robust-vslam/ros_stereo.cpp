#include <ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<opencv2/core/core.hpp>

int main(int argc, char **argv){
    ros::int(argc, argv, "ros_stereo");
    ros::start();
    
    
    return 0;
}


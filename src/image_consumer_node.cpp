#include <ros/ros.h>
#include "image_consumer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_consumer");
    ros::NodeHandle nh, private_nh("~"); 
    ImageConsumer consumer(nh, private_nh); 
    ros::spin(); 
    return 0;
}

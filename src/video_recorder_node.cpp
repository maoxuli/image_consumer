#include <ros/ros.h>
#include "video_recorder.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_recorder");
    ros::NodeHandle nh, private_nh("~"); 
    VideoRecorder recorder(nh, private_nh); 
    ros::spin(); 
    return 0;
}

#ifndef __VIDEO_RECORDER_H
#define __VIDEO_RECORDER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>

#include <boost/thread.hpp>
#include <memory>
#include <mutex>
#include <atomic>

class VideoRecorder
{
public: 
    VideoRecorder(const ros::NodeHandle& nh = ros::NodeHandle(), 
                  const ros::NodeHandle& private_nh = ros::NodeHandle("~")); 

    ~VideoRecorder(); 

private: 
    bool Open(); 
    void Close(); 
    bool Write(const cv::Mat& image); 

    bool reset_callback(std_srvs::Trigger::Request  &request, 
                        std_srvs::Trigger::Response &response);

    void image_callback(const sensor_msgs::Image::ConstPtr& image_msg);

private: 
    ros::NodeHandle _nh; 
    ros::NodeHandle _private_nh; 
    ros::Subscriber _image_sub; 
    
    ros::ServiceServer _reset_svr; 
    bool _auto_reset; 

    cv::VideoWriter _writer; 
}; 

#endif // #ifndef __VIDEO_RECORDER_H

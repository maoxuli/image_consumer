#ifndef __IMAGE_CONSUMER_H
#define __IMAGE_CONSUMER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>

#include <boost/thread.hpp>
#include <mutex>
#include <atomic>

#include "thread_safe_image.h"

class ImageConsumer
{
public: 
    ImageConsumer(const ros::NodeHandle& nh = ros::NodeHandle(), 
                  const ros::NodeHandle& private_nh = ros::NodeHandle("~")); 

    ~ImageConsumer(); 

private: 
    bool Open(); 
    void Close(); 
    bool Write(const cv::Mat& image); 

    void write_thread(); 

    bool reset_callback(std_srvs::Trigger::Request  &request, 
                        std_srvs::Trigger::Response &response);

    void image_callback(const sensor_msgs::Image::ConstPtr& image_msg);

private: 
    ros::NodeHandle _nh; 
    ros::NodeHandle _private_nh; 
    ros::Subscriber _image_sub; 
    ros::ServiceServer _reset_svr; 

    boost::thread _thread;
    std::atomic<bool> _stop; 

    cv::VideoWriter _writer; 
    std::mutex _mutex; 

    bool _auto_reset; 
    ThreadSafeImage _queued_image; 
}; 

#endif // #ifndef __IMAGE_CONSUMER_H

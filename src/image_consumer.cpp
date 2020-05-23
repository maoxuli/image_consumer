#include "image_consumer.h"
#include "ros_parameter.hpp"

#include <cv_bridge/cv_bridge.h>

ImageConsumer::ImageConsumer(const ros::NodeHandle& nh, 
                             const ros::NodeHandle& private_nh) 
: _nh(nh)
, _private_nh(private_nh)
{
    try 
    {
        std::string reset_service = "reset";
        ROS_INFO_STREAM("Advertise reset service: " << reset_service);
        _reset_svr = _nh.advertiseService(reset_service, &ImageConsumer::reset_callback, this);
        
        std::string image_topic = "image"; 
        ROS_INFO_STREAM("Subscribe image topic: " << image_topic);
        _image_sub = _nh.subscribe(image_topic, 2, &ImageConsumer::image_callback, this);
    }
    catch (const ros::Exception& ex)
    {
        ROS_ERROR_STREAM("ROS exception: " << ex.what());
        throw std::runtime_error(std::string("ROS exception: ") + ex.what()); 
    }

    try 
    {
        _stop = false; 
        ROS_INFO_STREAM("Start write thread...");
        _thread = boost::thread(boost::bind(&ImageConsumer::write_thread, this));
    } 
    catch (const std::exception& ex) 
    {
        ROS_ERROR_STREAM("Boost thread exception: " << ex.what());
        throw std::runtime_error(std::string("Boost thread exception: ") + ex.what());
    }
}

ImageConsumer::~ImageConsumer() 
{
    Close(); 

    if (_thread.joinable())
    {
        _stop = true; 
        _thread.join();
    } 
}

bool ImageConsumer::reset_callback(std_srvs::Trigger::Request &request, 
                                  std_srvs::Trigger::Response &response)
{
    try 
    {
        ROS_INFO("Reset service callback...");
        Close();
        Open(); 
        response.success = true; 
        return true; 
    }
    catch (const cv::Exception& ex)
    {
        ROS_ERROR_STREAM("OpenCV exception: " << ex.what());
        response.success = false; 
        response.message = std::string("OpenCV exception: ") + ex.what(); 
        return false;
    }
}

void ImageConsumer::image_callback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    try
    {
        ros::Time stamp = image_msg->header.stamp; 
        ROS_DEBUG("Image callback: %f", stamp.toSec());

        cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image; 
        _queued_image.set(image); 
    }
    catch(const cv_bridge::Exception& ex)
    {
        ROS_WARN_STREAM("cv_bridge exception: " << ex.what());
    }
    catch(const cv::Exception& ex)
    {
        ROS_WARN_STREAM("OpenCV exception: " << ex.what());
    }
    catch(const std::exception& ex)
    {
        ROS_WARN_STREAM("std::exception: " << ex.what());
    }
}

int get_fourcc(const std::string& format)
{
    int codec = 0; 
    codec = cv::VideoWriter::fourcc('X', '2', '6', '4');
    return codec; 
}

// Open stream for write 
// Refresh parameters for every open  
bool ImageConsumer::Open() 
{
    ROS_INFO("Opening stream...");
    std::lock_guard<std::mutex> lock(_mutex); 

    // auto reset on failure or end of stream 
    _auto_reset = true; 
    LoadParam(_private_nh, "auto_reset", _auto_reset); 

    // backend of OpenCV
    std::string backend; 
    LoadParam(_private_nh, "backend", backend);

    // stream target 
    std::string target;
    LoadParam(_private_nh, "target", target, true); 

    // target format 
    std::string format;
    LoadParam(_private_nh, "format", format, true); 

    // image size  
    int width = 0, height = 0; 
    LoadParam(_private_nh, "width", width); 
    LoadParam(_private_nh, "height", height); 

    // frame rate  
    double fps = 0; 
    LoadParam(_private_nh, "fps", fps); 

    ROS_INFO_STREAM("Stream target: " << target);
    if (backend == "gstreamer") {
        _writer.open(target, cv::CAP_GSTREAMER, get_fourcc(format), fps, cv::Size(width, height));
    }
    else {
        _writer.open(target, get_fourcc(format), fps, cv::Size(width, height)); 
    }

    if (!_writer.isOpened()) 
    {
        ROS_ERROR_STREAM("Failed to open stream target: " << target);
        return false;
    }

    ROS_INFO("Stream opened.");
    return true;
}

void ImageConsumer::Close() 
{
    ROS_INFO("Closing stream...");
    std::lock_guard<std::mutex> lock(_mutex); 
    _writer.release(); 
    ROS_INFO("Stream closed!");
}

bool ImageConsumer::Write(const cv::Mat& image)
{
    ROS_DEBUG("Write image to stream...");
    std::lock_guard<std::mutex> lock(_mutex); 
    if (!_writer.isOpened()) return false; 
    _writer.write(image);
    return true;  
}

void ImageConsumer::write_thread() 
{  
    ROS_INFO("Write thread start...");
    Open(); 

    double start_time = ros::Time::now().toNSec(); 
    int frame_count = 0; 
    while (!_stop && ros::ok()) 
    {
        try 
        {
            if (!Write(_queued_image.pop())) 
            {
                ROS_WARN("Failed to write image!");
                ros::Duration(0.5).sleep();
                if (_auto_reset) 
                {
                    Close(); 
                    ros::Duration(0.5).sleep();
                    Open(); 
                }
                continue; 
            }
        }
        catch (const cv::Exception& ex)
        {
            ROS_ERROR_STREAM("OpenCV exception: " << ex.what());
            continue; 
        }

        frame_count++; 
        double stop_time = ros::Time::now().toNSec(); 
        if (stop_time - start_time > 1000000000)
        {
            start_time = stop_time; 
            ROS_INFO_STREAM("fps: " << frame_count); 
            frame_count = 0; 
        }
    }

    Close(); 
    ROS_INFO("Write thread stopped!");
}

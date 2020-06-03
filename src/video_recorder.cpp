#include "video_recorder.h"
#include "ros_parameter.hpp"

#include <cv_bridge/cv_bridge.h>

namespace {
int get_fourcc(const std::string& format)
{
    int codec = 0;
    if (format.size() >= 4) 
    {
        ROS_INFO_STREAM("fourcc: " << format[0] << "," << format[1] << "," << format[2] << "," << format[3]); 
        codec = cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]);
    } 
    ROS_INFO_STREAM("fourcc: " << codec); 
    return codec; 
}
}

VideoRecorder::VideoRecorder(const ros::NodeHandle& nh, 
                             const ros::NodeHandle& private_nh) 
: _nh(nh)
, _private_nh(private_nh)
{
    try 
    {
        std::string reset_service = "reset";
        ROS_INFO_STREAM("Advertise reset service: " << reset_service);
        _reset_svr = _nh.advertiseService(reset_service, &VideoRecorder::reset_callback, this);
        
        std::string image_topic = "image"; 
        ROS_INFO_STREAM("Subscribe image topic: " << image_topic);
        _image_sub = _nh.subscribe(image_topic, 2, &VideoRecorder::image_callback, this);

        Open(); 
    }
    catch (const ros::Exception& ex)
    {
        ROS_ERROR_STREAM("ROS exception: " << ex.what());
        throw std::runtime_error(std::string("ROS exception: ") + ex.what()); 
    }
}

VideoRecorder::~VideoRecorder() 
{
    Close(); 
}

bool VideoRecorder::reset_callback(std_srvs::Trigger::Request &request, 
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

void VideoRecorder::image_callback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    static double start_time = ros::Time::now().toNSec(); 
    static int frame_count = 0; 

    try
    {
        ros::Time stamp = image_msg->header.stamp; 
        ROS_DEBUG("Image callback: %f", stamp.toSec());

        cv::Mat image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image; 
        Write(image); 
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

    frame_count++; 
    double stop_time = ros::Time::now().toNSec(); 
    if (stop_time - start_time > 1000000000)
    {
        start_time = stop_time; 
        ROS_INFO_STREAM("Wirte FPS: " << frame_count); 
        frame_count = 0; 
    }
}

// Open stream for write 
// Refresh parameters for every open  
bool VideoRecorder::Open() 
{
    ROS_INFO("Opening output stream...");

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

    ROS_INFO_STREAM("Output stream target: " << target);
    if (backend == "gstreamer") {
        _writer.open(target, cv::CAP_GSTREAMER, get_fourcc(format), fps, cv::Size(width, height));
        ROS_INFO("Output stream with gstreamer backend opened.");
    }
    else {
        _writer.open(target, get_fourcc(format), fps, cv::Size(width, height)); 
        ROS_INFO("Output stream opened.");
    }

    if (!_writer.isOpened()) 
    {
        ROS_ERROR_STREAM("Failed to open output stream target: " << target);
        return false;
    }

    return true;
}

void VideoRecorder::Close() 
{
    ROS_INFO("Closing output stream...");
    _writer.release(); 
    ROS_INFO("Output stream closed!");
}

bool VideoRecorder::Write(const cv::Mat& image)
{
    ROS_DEBUG("Write image to stream...");
    if (image.empty()) return false; 

    if (!_writer.isOpened()) 
    {
        if (!_auto_reset || !Open()) 
        {
            return false;
        } 
    }
    _writer.write(image);
    return true;  
}

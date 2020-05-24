#ifndef __THREAD_SAFE_IMAGE_H
#define __THREAD_SAFE_IMAGE_H 

#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable> 
#include <atomic>

class ThreadSafeImage
{
public: 
    ThreadSafeImage();
    ~ThreadSafeImage(); 

    void set(const cv::Mat& image);
    cv::Mat get();
    cv::Mat pop();

private: 
    cv::Mat _image;
    std::mutex _mutex;
    std::condition_variable _condition;
    std::atomic<bool> _stop; 
};

#endif // #ifndef __THREAD_SAFE_IMAGE_H

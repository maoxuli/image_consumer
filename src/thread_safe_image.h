#ifndef __THREAD_SAFE_IMAGE_H
#define __THREAD_SAFE_IMAGE_H 

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

class ThreadSafeImage
{
  boost::mutex _mutex;
  boost::condition_variable _condition;
  cv::Mat _image;

public:
  void set(const cv::Mat& image);
  cv::Mat get();
  cv::Mat pop();
};

#endif // #ifndef __THREAD_SAFE_IMAGE_H

#include "thread_safe_image.h"

void ThreadSafeImage::set(const cv::Mat& image)
{
    boost::unique_lock<boost::mutex> lock(_mutex);
    _image = image;
    _condition.notify_one();
}

cv::Mat ThreadSafeImage::get()
{
    boost::unique_lock<boost::mutex> lock(_mutex);
    return _image;
}

cv::Mat ThreadSafeImage::pop()
{
    cv::Mat image;
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        while (_image.empty())
        {
            _condition.wait(lock);
        }
        image = _image;
        _image.release();
    }
    return image;
}

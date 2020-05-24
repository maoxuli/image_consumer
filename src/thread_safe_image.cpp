#include "thread_safe_image.h"

ThreadSafeImage::ThreadSafeImage() 
: _stop(false)
{

}

ThreadSafeImage::~ThreadSafeImage()
{
    _stop = true; 
    _condition.notify_all(); 
}

void ThreadSafeImage::set(const cv::Mat& image)
{
    std::unique_lock<std::mutex> lock(_mutex);
    _image = image;
    _condition.notify_one();
}

cv::Mat ThreadSafeImage::get()
{
    std::unique_lock<std::mutex> lock(_mutex);
    return _image;
}

cv::Mat ThreadSafeImage::pop()
{
    cv::Mat image;
    {
        std::unique_lock<std::mutex> lock(_mutex);
        while (!_stop && _image.empty())
        {
            _condition.wait(lock);
        }
        image = _image;
        _image.release();
    }
    return image;
}

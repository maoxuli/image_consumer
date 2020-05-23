#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "image_consumer.h"

namespace image_consumer
{

class ImageConsumerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        _consumer.reset(new ImageConsumer(getNodeHandle(), getPrivateNodeHandle()));  
    }

private:
    boost::shared_ptr<ImageConsumer> _consumer;
};

} // namespace image_consumer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_consumer::ImageConsumerNodelet, nodelet::Nodelet)

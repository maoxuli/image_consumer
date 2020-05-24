#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "video_recorder.h"

namespace video_recorder
{

class VideoRecorderNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        _recorder.reset(new VideoRecorder(getNodeHandle(), getPrivateNodeHandle()));  
    }

private:
    boost::shared_ptr<VideoRecorder> _recorder;
};

} // namespace video_recorder

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_recorder::VideoRecorderNodelet, nodelet::Nodelet)

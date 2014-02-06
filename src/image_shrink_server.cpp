#include <image_shrink/image_shrink_server.h>

namespace image_shrink {
    ImageShrinkServer::ImageShrinkServer(ros::NodeHandle& node) : _n(node)
    {
        _pub = _n.advertise<>(
    }

    void checkSubscribers()
    {
        if(_pub.getNumSubscribers() == 0) {
            if(_sub) _sub.shutdown();
        } else {
            if (!_sub){
                _sub = _n.subscrib("image_in", 1, &ImageShrinkServer::imageCallback, this);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "image_shrink_server");
    ImageShrinkServer server;
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        server.checkSubscribers();
        r.sleep();
    }

    return 0;
}

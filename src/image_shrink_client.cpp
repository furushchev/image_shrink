#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_shrink/BinaryImage.h>

namespace enc = sensor_msgs::image_encodings;

class ImageShrinkClient {
    ros::NodeHandle _n;
    ros::NodeHandle _ln;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    sensor_msgs::ImagePtr _img_ptr;
    bool _isInitialized;

public:
    ImageShrinkClient() : _ln(ros::NodeHandle("~")), _isInitialized(false) {
        _pub = _n.advertise<sensor_msgs::Image>("image_edged", 10);
        _sub = _n.subscribe<image_shrink::BinaryImage>("image_binary", 1, &ImageShrinkClient::binaryCallback, this);
    }
    virtual ~ImageShrinkClient(){};

    void initialize(){
        _img_ptr = boost::make_shared<sensor_msgs::Image>();
    }

    void binaryCallback(const image_shrink::BinaryImageConstPtr& msg){
        if (!_isInitialized){
            initialize();
            _isInitialized = true;
        }
        _img_ptr->height = msg->height;
        _img_ptr->width = msg->width;
        _img_ptr->encoding = "mono8";
        _img_ptr->step = msg->width;
        _img_ptr->data.resize(_img_ptr->height * _img_ptr->width);
        for (int y = 0; y < (int)_img_ptr->height; ++y){
            for (int x = 0; x < (int)_img_ptr->width; ++x){
                int i = y * _img_ptr->width + x;
                if (((msg->data[i/8] >> (i%8)) & 1) == 1){
                    _img_ptr->data[i] = 255;
                } else {
                    _img_ptr->data[i] = 0;
                }
            }
        }
        _pub.publish(_img_ptr);
    } // end of callback
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_shrink_client");
    ImageShrinkClient client;
    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

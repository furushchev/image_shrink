#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_shrink/SparseBinaryImage.h>

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
        _sub = _n.subscribe<image_shrink::SparseBinaryImage>("image_binary_sparse", 1, &ImageShrinkClient::binaryCallback, this);
    }
    virtual ~ImageShrinkClient(){};

    void initialize(){
        _img_ptr = boost::make_shared<sensor_msgs::Image>();
    }

    void binaryCallback(const image_shrink::SparseBinaryImageConstPtr& msg){
        if (!_isInitialized){
            initialize();
            _isInitialized = true;
        }
        _img_ptr->height = msg->height;
        _img_ptr->width = msg->width;
        _img_ptr->encoding = "mono8";
        _img_ptr->step = msg->width;
        _img_ptr->data.resize(_img_ptr->height * _img_ptr->width);
        for (int x = 0; x < msg->width; ++x){
            for (int y = 0; y < msg->height; ++y){
                _img_ptr->data[y * msg->width * x] = 0;
            }
        }

        for (int i = 0; i < msg->data.size(); ++i){
            uint16_t pos = msg->data[i];
            uint8_t x = (uint8_t)(pos & 65280);
            uint8_t y = (uint8_t)(pos >> 8);
            _img_ptr->data[y * _img_ptr->width + x] = 255;
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

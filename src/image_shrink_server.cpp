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

class ImageShrinkServer {
    ros::NodeHandle _n;
    ros::NodeHandle _ln;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    cv_bridge::CvImagePtr _resized_img_ptr;
    double _rate, _imageScale;
    double _canny_threshold1, _canny_threshold2;
    bool _isFirstCallback, _isInitialized;

public:
    ImageShrinkServer() : _ln(ros::NodeHandle("~")), _isFirstCallback(false), _isInitialized(false) {
//        _pub = _n.advertise<image_shrink::BinaryImage>("image_binary", 10);
        _pub = _n.advertise<sensor_msgs::Image>("image_debug", 10);
    }
    virtual ~ImageShrinkServer(){};

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        } catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (!_isInitialized){
            initialize();
            _isInitialized = true;
        }

        if(!_isFirstCallback){
            _resized_img_ptr = boost::make_shared<cv_bridge::CvImage>();
            _resized_img_ptr->encoding = "mono8";
            _isFirstCallback = true;
        }

        cv::Size dsize(round(cv_ptr->image.cols * _imageScale), round(cv_ptr->image.rows * _imageScale));
        cv::resize(cv_ptr->image, _resized_img_ptr->image, dsize);
        cv::blur(_resized_img_ptr->image, _resized_img_ptr->image, cv::Size(3,3));
        cv::Canny(_resized_img_ptr->image, _resized_img_ptr->image, _canny_threshold1, _canny_threshold2);
        _pub.publish(_resized_img_ptr->toImageMsg());
    }

    void initialize(){
        _ln.param("scale", _imageScale, 0.4);
        _ln.param("rate", _rate, 3.0);
        _ln.param("canny_threshold1", _canny_threshold1, 100.0);
        _ln.param("canny_threshold2", _canny_threshold2, 200.0);
        ROS_INFO("initialized scale: %lf, rate: %lf, threshold1: %lf, threshold2: %lf", _imageScale, _rate, _canny_threshold1, _canny_threshold2);
    }
\
    void checkSubscribers(){
        if(_pub.getNumSubscribers() == 0) {
            if(_sub) {
                _sub.shutdown();
                ROS_INFO("unsubscribe");
            }
        } else {
            if (!_sub){
                _sub = _n.subscribe("image_in", 1, &ImageShrinkServer::imageCallback, this);
                ROS_INFO("subscribe");
            }
        }
    }



}; // end of ImageShrinkServer class definition


int main(int argc, char *argv[]){
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

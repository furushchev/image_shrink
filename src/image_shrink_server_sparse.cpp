#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_shrink/BinaryImage.h>
#include <image_shrink/SparseBinaryImage.h>

namespace enc = sensor_msgs::image_encodings;

class ImageShrinkServer {
    ros::NodeHandle _n;
    ros::NodeHandle _ln;
    ros::Publisher _pub_raw, _pub_bin, _pub_spr;
    ros::Subscriber _sub;
    cv_bridge::CvImagePtr _resized_img_ptr;
    image_shrink::BinaryImagePtr _bin_img_ptr;
    image_shrink::SparseBinaryImagePtr _spr_img_ptr;
    double _rate, _imageScale;
    double _canny_threshold1, _canny_threshold2;
    bool _isFirstCallback, _isInitialized;

public:
    ImageShrinkServer() : _ln(ros::NodeHandle("~")), _isFirstCallback(false), _isInitialized(false) {
        _pub_bin = _n.advertise<image_shrink::BinaryImage>("image_binary", 10);
        _pub_spr = _n.advertise<image_shrink::SparseBinaryImage>("image_binary_sparse", 10);
        _pub_raw = _n.advertise<sensor_msgs::Image>("image_edge_raw", 10);

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
            _bin_img_ptr = boost::make_shared<image_shrink::BinaryImage>();
            _spr_img_ptr = boost::make_shared<image_shrink::SparseBinaryImage>();
            _resized_img_ptr->encoding = "mono8";
            _isFirstCallback = true;
        }

        cv::Size dsize(round(cv_ptr->image.cols * _imageScale), round(cv_ptr->image.rows * _imageScale));
        cv::resize(cv_ptr->image, _resized_img_ptr->image, dsize);
        cv::blur(_resized_img_ptr->image, _resized_img_ptr->image, cv::Size(3,3));
        cv::Canny(_resized_img_ptr->image, _resized_img_ptr->image, _canny_threshold1, _canny_threshold2);
        _pub_raw.publish(_resized_img_ptr->toImageMsg());

        //  binary
        _bin_img_ptr->width = _resized_img_ptr->image.cols;
        _bin_img_ptr->height = _resized_img_ptr->image.rows;
        _bin_img_ptr->data.resize(_bin_img_ptr->width * _bin_img_ptr->height / 8 + 1);
        for (int y = 0; y < _resized_img_ptr->image.rows; ++y){
            for (int x = 0; x < _resized_img_ptr->image.cols; ++x){
                if (_resized_img_ptr->image.at<uchar>(y,x) > 0) {
                    int i = y * _resized_img_ptr->image.cols + x;
                    _bin_img_ptr->data[i/8] |= (1 << (i%8));
                }
            }
        }
        _pub_bin.publish(*_bin_img_ptr);

        //  binary sparse
        _spr_img_ptr->width = (uint8_t)_resized_img_ptr->image.cols;
        _spr_img_ptr->height = (uint8_t)_resized_img_ptr->image.rows;
        if (!_spr_img_ptr->data.empty()) _spr_img_ptr->data.clear();
        for (uint8_t y = 0; y < _spr_img_ptr->height; ++y){
            for (uint8_t x = 0; x < _spr_img_ptr->width; ++x){
                if (_resized_img_ptr->image.at<uchar>(y,x) > 0) {
                    uint16_t i = (y << 8) | x;
                    _spr_img_ptr->data.push_back(i);
                }
            }
        }
        _pub_spr.publish(*_spr_img_ptr);

        // sleep
        ros::Rate pubRate(_rate);
        pubRate.sleep();
    } // end of callback function

    void initialize(){
        _ln.param("scale", _imageScale, 0.4);
        _ln.param("rate", _rate, 3.0);
        _ln.param("canny_threshold1", _canny_threshold1, 100.0);
        _ln.param("canny_threshold2", _canny_threshold2, 200.0);
        ROS_INFO("initialized scale: %.1lf, rate: %.1lf, threshold1: %.1lf, threshold2: %.1lf", _imageScale, _rate, _canny_threshold1, _canny_threshold2);
    }

    void checkSubscribers(){
        if(_pub_raw.getNumSubscribers() == 0 && _pub_bin.getNumSubscribers() == 0 && _pub_spr.getNumSubscribers() == 0) {
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

    double rate(){
        return this->_rate;
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

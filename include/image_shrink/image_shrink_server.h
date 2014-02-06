#ifndef IMAGE_SHRINK_SERVER_H_
#define IMAGE_SHRINK_SERVER_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_shrink/EdgedImage.h>

namespace image_shrink {
    class ImageShrinkServer
    {
        ros::NodeHandle _n;
        ros::Publisher _pub;
        ros::Subscriber _sub;

        float rate, imageScale;
        double canny_threshold1, canny_threshold2;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    public:
        void initialize();
        void checkSubscribers();

        ImageShrinkServer(ros::NodeHandle& node);
        virtual ~ImageShrinkServer();
    };

#endif

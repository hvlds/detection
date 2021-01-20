#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

class FilteredCamera {
   public:
    bool is_ir = true;
    bool needs_change = false;
    int count = 0;
    int count_limit = 40;
    image_transport::Publisher image_pub;
    ros::Publisher camera_info_pub;

    void irCallback(const sensor_msgs::ImageConstPtr& img);
    void rgbCallback(const sensor_msgs::ImageConstPtr& img);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
};
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
   private:
    bool is_ir = false;
    bool needs_change = false;
    int count = 0;
    int count_limit = 40;

    std::string ir_camera_name;
    std::string rgb_camera_name;
    std::string filtered_camera_name;
    std::string raw_image_topic;
    std::string filtered_image_topic;

    image_transport::Publisher image_pub;
    ros::Publisher camera_info_pub;
    ros::Publisher ir_flag_pub;
    ros::Subscriber image_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber toggle_camera_sub;
    ros::NodeHandle* nh;

   public:
    FilteredCamera(ros::NodeHandle* nh, image_transport::ImageTransport* it);
    void ir_callback(const sensor_msgs::ImageConstPtr& img);
    void rgb_callback(const sensor_msgs::ImageConstPtr& img);
    void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void toggle_camera_callback(const std_msgs::Bool::ConstPtr& msg);
    void toggle_camera_subscription();
};
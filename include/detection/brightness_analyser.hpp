#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

class BrightnessAnalyser {
   private:
    ros::NodeHandle* nh;
    ros::Subscriber image_sub;
    ros::Subscriber arduino_sub;
    ros::Subscriber ir_state_sub;

    int arduino_brightness = -1;
    int camera_brightness = -1;

    int count = 0;
    int count_limit = 30;
    bool needs_change = false;
    bool ir_state;

   public:
    BrightnessAnalyser(ros::NodeHandle* nh);
    void ir_state_callback(const std_msgs::Bool::ConstPtr& msg);
    void arduino_callback(const std_msgs::Int32::ConstPtr& msg);
    void image_callback(const sensor_msgs::ImageConstPtr& img);
};
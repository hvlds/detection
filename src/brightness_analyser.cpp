#include "detection/brightness_analyser.hpp"
#include <iostream>

BrightnessAnalyser::BrightnessAnalyser(ros::NodeHandle* nh) {
    this->nh = nh;

    std::string arduino_topic;
    if (nh->getParam("/detection/arduino_topic", arduino_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/arduino_topic'");
        arduino_topic = "/brightness";
    }

    std::string camera_name;
    if (nh->getParam("/detection/filtered_camera_name", camera_name) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_camera_name'");
        camera_name = "/filtered_camera";
    }

    std::string image_topic;
    if (nh->getParam("/detection/filtered_image_topic", image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        image_topic = "/image";
    }

    this->image_sub = nh->subscribe(
        camera_name + image_topic, 10, &BrightnessAnalyser::image_callback, this);
    this->arduino_sub = nh->subscribe(
        arduino_topic, 10, &BrightnessAnalyser::arduino_callback, this);
    this->ir_state_sub = nh->subscribe(
        camera_name + "/is_ir", 10, &BrightnessAnalyser::ir_state_callback, this);
}

void BrightnessAnalyser::ir_state_callback(const std_msgs::Bool::ConstPtr&  msg) {
    this->ir_state = msg->data;
}

void BrightnessAnalyser::arduino_callback(const std_msgs::Int32::ConstPtr& msg) {
    this->arduino_brightness = msg->data;
    ROS_INFO("Arduino Brightness: %i", this->arduino_brightness);
}

void BrightnessAnalyser::image_callback(const sensor_msgs::ImageConstPtr& img) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

    cv::Mat img_rgb(480, 640, CV_8UC3);
    cv_ptr->image.convertTo(img_rgb, CV_8UC3);

    // Convert to HSV in order to extract Brightness
    cv::Mat img_hsv;
    cv::cvtColor(img_rgb, img_hsv, CV_RGB2HSV);

    cv::Scalar m = cv::mean(img_hsv);
    this->camera_brightness = static_cast<int>(m[2]);

    ROS_INFO("Camera Brightness: %i", this->camera_brightness);

    this->count++;
    if (this->count > this->count_limit) {
        this->count = 0;
        this->needs_change = true;
    }
}
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

image_transport::Publisher image_pub;
int brightness = -1;

void arduinoCallback(const std_msgs::Int32::ConstPtr& msg) {
    // Assigned value read from the Arduino
    brightness = msg->data;
    ROS_INFO("brightness: %i", brightness);
}

void imageCallback(const sensor_msgs::ImageConstPtr& img) {
    // Recieve IR frame from camera in MONO16 (GRAY16 in OpenNI2) format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    
    cv::Mat img8(480, 640, CV_8UC1);
    cv_ptr->image.convertTo(img8, CV_8UC1);

    // Convert image gray -> rgb -> hsv
    cv::Mat img_rgb, img_hsv;
    cv::cvtColor(img8, img_rgb, CV_GRAY2RGB);
    cv::cvtColor(img_rgb, img_hsv, CV_RGB2HSV);

    cv::Scalar m = cv::mean(img_hsv);
    ROS_INFO("camera_brightness %f", m[2]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "analyser");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    std::string arduino_topic;
    if (n.getParam("/detection/arduino_topic", arduino_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/arduino_topic'");
        arduino_topic = "/brightness";
    }

    std::string filtered_image_topic;
    if (n.getParam("/detection/filtered_image_topic", filtered_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        filtered_image_topic = "/camera/ir/image_filtered";
    }

    ros::Subscriber filtered_image_sub = n.subscribe(filtered_image_topic, 10, imageCallback);
    ros::Subscriber arduino_sub = n.subscribe(arduino_topic, 10, arduinoCallback);

    image_pub = it.advertise(filtered_image_topic, 1);

    ros::spin();

    return 0;
}
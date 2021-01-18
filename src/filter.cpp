#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <iostream>

image_transport::Publisher image_pub;
int brightness = -1;

void arduinoCallback(const std_msgs::Int32::ConstPtr& msg) {
    // Assigned value read from the Arduino
    brightness = msg->data;
}

void imageCallback(const sensor_msgs::ImageConstPtr& img) {
    // Recieve IR frame from camera in MONO16 (GRAY16 in OpenNI2) format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    cv::Mat img8(480, 640, CV_8UC1);
    cv_ptr->image.convertTo(img8, CV_8UC1);

    // Publish the image in new topic (can be processed from rqt_image_view)
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();

    // Publish the new formatted image with same header as the input image
    msg->header = img->header; 
    image_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Assign parsed parameters from ./config/detection/general.yaml
    std::string camera_topic;
    if(n.getParam("/detection/camera_topic", camera_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/camera_topic'");
        camera_topic = "/camera/ir/image_raw";
    }

    std::string arduino_topic;
    if(n.getParam("/detection/arduino_topic", arduino_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/arduino_topic'");
        arduino_topic = "/brightness";
    }

    std::string filtered_image_topic;
    if(n.getParam("/detection/filtered_image_topic", filtered_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        filtered_image_topic = "/camera/ir/image_filtered";
    }

    ros::Subscriber image_sub = n.subscribe(camera_topic, 10, imageCallback);
    ros::Subscriber arduino_sub = n.subscribe(arduino_topic, 10, arduinoCallback);

    image_pub = it.advertise(filtered_image_topic, 1);

    ros::spin();

    return 0;
}
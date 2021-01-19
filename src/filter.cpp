#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

image_transport::Publisher image_pub;

void infraredCallback(const sensor_msgs::ImageConstPtr& img) {
    // Recieve IR frame from camera in MONO16 (GRAY16 in OpenNI2) format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    cv::Mat img8(480, 640, CV_8UC1);
    cv_ptr->image.convertTo(img8, CV_8UC1);

    // Convert from gray to rgb
    cv::Mat img_rgb;
    cv::cvtColor(img8, img_rgb, CV_GRAY2RGB);

    // Publish the image in new topic (can be processed from rqt_image_view)
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_rgb).toImageMsg();

    // Publish the new formatted image with same header as the input image
    msg->header = img->header;
    image_pub.publish(msg);
}

void rgbCallback(const sensor_msgs::ImageConstPtr& img) {
    // Recieve RGB frame from camera in RGB8 format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    cv::Mat img_rgb(480, 640, CV_8UC3);
    cv_ptr->image.convertTo(img_rgb, CV_8UC3);

    // Publish the image in new topic (can be processed from rqt_image_view)
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_rgb).toImageMsg();

    // Publish the new formatted image with same header as the input image
    msg->header = img->header;
    image_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Assign parsed parameters from ./config/detection/general.yaml
    std::string ir_camera_name;
    if (n.getParam("/detection/ir_camera_name", ir_camera_name) == false) {
        ROS_ERROR("Failed to get param '/detection/ir_camera_name'");
        ir_camera_name = "/camera/ir";
    }

    std::string rgb_camera_name;
    if (n.getParam("/detection/rgb_camera_name", rgb_camera_name) == false) {
        ROS_ERROR("Failed to get param '/detection/rgb_camera_name'");
        rgb_camera_name = "/camera/rgb";
    }

    std::string raw_image_topic;
    if (n.getParam("/detection/raw_image_topic", raw_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/raw_image_topic'");
        raw_image_topic = "/image_raw";
    }

    std::string filtered_image_topic;
    if (n.getParam("/detection/filtered_image_topic", filtered_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        filtered_image_topic = "/image_filtered";
    }

    ros::Subscriber ir_image_sub = n.subscribe(
        ir_camera_name + raw_image_topic, 10, infraredCallback);
    // ros::Subscriber rgb_image_sub = n.subscribe(
    //     rgb_camera_name + raw_image_topic, 10, rgbCallback);

    image_pub = it.advertise(ir_camera_name + filtered_image_topic, 1);

    ros::spin();

    return 0;
}
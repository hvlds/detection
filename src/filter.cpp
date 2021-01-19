#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

image_transport::Publisher image_pub;
ros::Subscriber image_sub;

ros::Publisher camera_info_pub;
ros::Subscriber camera_info_sub;

bool is_ir = true;
bool needs_change = false;
int count = 0;
int count_limit = 20;

void irCallback(const sensor_msgs::ImageConstPtr& img);
void rgbCallback(const sensor_msgs::ImageConstPtr& img);

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Rate rate(10);

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

    std::string filtered_camera_name;
    if (n.getParam("/detection/filtered_camera_name", filtered_camera_name) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_camera_name'");
        rgb_camera_name = "/filtered_camera";
    }

    std::string raw_image_topic;
    if (n.getParam("/detection/raw_image_topic", raw_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/raw_image_topic'");
        raw_image_topic = "/image_raw";
    }

    std::string filtered_image_topic;
    if (n.getParam("/detection/filtered_image_topic", filtered_image_topic) == false) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        filtered_image_topic = "/image";
    }

    image_sub = n.subscribe(
        ir_camera_name + raw_image_topic, 1, irCallback);
    image_pub = it.advertise(
        filtered_camera_name + filtered_image_topic, 1);
    camera_info_pub = n.advertise<sensor_msgs::CameraInfo>(
        filtered_camera_name + "/camera_info", 1);
                
    while (ros::ok()) {
        if (is_ir == false && needs_change == true) {
            image_sub.shutdown();
            image_sub = n.subscribe(
                rgb_camera_name + raw_image_topic, 1, rgbCallback);
            needs_change = false;
        } else if (is_ir == true && needs_change == true) {
            image_sub.shutdown();
            image_sub = n.subscribe(
                ir_camera_name + raw_image_topic, 1, irCallback);
            needs_change = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void irCallback(const sensor_msgs::ImageConstPtr& img) {
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

    count++;
    if (count > count_limit) {
        count = 0;
        is_ir = false;
        needs_change = true;
    }
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

    count++;
    if (count > count_limit) {
        count = 0;
        is_ir = true;
        needs_change = true;
    }
}

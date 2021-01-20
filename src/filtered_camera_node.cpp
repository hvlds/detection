#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "message_filters/subscriber.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "detection/filtered_camera.hpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "filtered_camera");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Publisher ir_flag_pub;
    ros::Rate rate(10);

    FilteredCamera filtered_camera = FilteredCamera();
    
    ros::Subscriber image_sub;
    ros::Subscriber camera_info_sub;

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

    // Image publisher and subscriber
    image_sub = n.subscribe(
        ir_camera_name + raw_image_topic, 1, &FilteredCamera::irCallback, &filtered_camera);
    filtered_camera.image_pub = it.advertise(
        filtered_camera_name + filtered_image_topic, 1);

    // Camera Info publisher and subscriber
    camera_info_sub = n.subscribe(
        ir_camera_name + "/camera_info", 1, &FilteredCamera::cameraInfoCallback, &filtered_camera);
    filtered_camera.camera_info_pub = n.advertise<sensor_msgs::CameraInfo>(
        filtered_camera_name + "/camera_info", 1);

    // Publisher of the flag with the information if the IR stream is active
    ir_flag_pub = n.advertise<std_msgs::Bool>(
        filtered_camera_name + "/is_ir", 1
    );

    while (ros::ok()) {
        std_msgs::Bool ir_msg;
        ir_msg.data = filtered_camera.is_ir;
        ir_flag_pub.publish(ir_msg);

        if (!filtered_camera.is_ir && filtered_camera.needs_change) {
            image_sub.shutdown();
            camera_info_sub.shutdown();

            image_sub = n.subscribe(
                rgb_camera_name + raw_image_topic, 1, &FilteredCamera::rgbCallback, &filtered_camera);
            camera_info_sub = n.subscribe(
                rgb_camera_name + "/camera_info", 1, &FilteredCamera::cameraInfoCallback, &filtered_camera);

            filtered_camera.needs_change = false;
        } else if (filtered_camera.is_ir && filtered_camera.needs_change) {
            image_sub.shutdown();
            camera_info_sub.shutdown();

            image_sub = n.subscribe(
                ir_camera_name + raw_image_topic, 1, &FilteredCamera::irCallback, &filtered_camera);
            camera_info_sub = n.subscribe(
                ir_camera_name + "/camera_info", 1, &FilteredCamera::cameraInfoCallback, &filtered_camera);

            filtered_camera.needs_change = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

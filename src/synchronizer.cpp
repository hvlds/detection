#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "synchronizer");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    std::string filtered_image_topic;
    n.getParam("/detection/filtered_image_topic", filtered_image_topic);

    std::string camera_info_topic;
    n.getParam("/detection/camera_info", camera_info_topic);

    image_transport::SubscriberFilter image_sub(it, filtered_image_topic, 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, camera_info_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);

    ros::spin();

    return 0;
}
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
    ros::init(argc, argv, "new_camera");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    image_transport::SubscriberFilter image_sub(it, "/camera/ir/image_filtered", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, "/camera/ir/camera_info", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);

    ros::spin();

    return 0;
}
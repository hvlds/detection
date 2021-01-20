#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>

#include "detection/filtered_camera.hpp"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "filtered_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate rate(10);

    FilteredCamera filtered_camera = FilteredCamera(&nh, &it);

    while (ros::ok()) {
        filtered_camera.toggle_camera_subscription();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

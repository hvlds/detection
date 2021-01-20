#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "detection/brightness_analyser.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "brightness_analyser");
    ros::NodeHandle nh;

    BrightnessAnalyser brightness_analyser = BrightnessAnalyser(&nh);
    ROS_INFO("PIP");
    ros::spin();

    return 0;
}
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void chatterCallback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    cv::Mat img8(800, 600, CV_8UC1);
    cv_ptr->image.convertTo(img8, CV_8UC1);
    cv::imshow("8-bit Mono", img8);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/ir/image_raw", 1000, chatterCallback);
    ros::spin();

    return 0;
}
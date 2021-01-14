#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

image_transport::Publisher image_pub;

void chatterCallback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    cv::Mat img8(480, 640, CV_8UC1);
    cv_ptr->image.convertTo(img8, CV_8UC1);
    cv::imshow("8-bit Mono", img8);
    
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
    image_pub.publish(msg);

    //cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;

    std::string camera_topic;
    n.param<std::string>("camera_topic", camera_topic, "/camera/ir/image_raw");

    image_transport::ImageTransport it(n);
    image_pub = it.advertise("filtered_camera/image", 1);

    ros::Subscriber sub = n.subscribe(camera_topic, 1000, chatterCallback);
    ros::spin();

    return 0;
}
#include "detection/filtered_camera.hpp"

void FilteredCamera::irCallback(const sensor_msgs::ImageConstPtr& img) {
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
    this->image_pub.publish(msg);

    this->count++;
    if (this->count > this->count_limit) {
        this->count = 0;
        this->is_ir = false;
        this->needs_change = true;
    }
}

void FilteredCamera::rgbCallback(const sensor_msgs::ImageConstPtr& img) {
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
    this->image_pub.publish(msg);

    this->count++;
    if (this->count > this->count_limit) {
        this->count = 0;
        this->is_ir = true;
        this->needs_change = true;
    }
}

void FilteredCamera::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info) {
    sensor_msgs::CameraInfo camera_info;
    this->camera_info_pub.publish(*info);
}
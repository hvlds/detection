#include "detection/filtered_camera.hpp"

FilteredCamera::FilteredCamera(ros::NodeHandle* nh, image_transport::ImageTransport* it) {
    this->nh = nh;

    // Assign parsed parameters from ./config/detection/general.yaml
    if (!this->nh->getParam("/detection/ir_camera_name", this->ir_camera_name)) {
        ROS_ERROR("Failed to get param '/detection/ir_camera_name'");
        this->ir_camera_name = "/camera/ir";
    }

    if (!this->nh->getParam("/detection/rgb_camera_name", this->rgb_camera_name)) {
        ROS_ERROR("Failed to get param '/detection/rgb_camera_name'");
        this->rgb_camera_name = "/camera/rgb";
    }

    if (!this->nh->getParam("/detection/filtered_camera_name", this->filtered_camera_name)) {
        ROS_ERROR("Failed to get param '/detection/filtered_camera_name'");
        this->rgb_camera_name = "/filtered_camera";
    }

    if (!this->nh->getParam("/detection/raw_image_topic", this->raw_image_topic)) {
        ROS_ERROR("Failed to get param '/detection/raw_image_topic'");
        this->raw_image_topic = "/image_raw";
    }

    if (!this->nh->getParam("/detection/filtered_image_topic", this->filtered_image_topic)) {
        ROS_ERROR("Failed to get param '/detection/filtered_image_topic'");
        this->filtered_image_topic = "/image";
    }

    this->image_pub = it->advertise(
        this->filtered_camera_name + this->filtered_image_topic, 1);
    this->camera_info_pub = this->nh->advertise<sensor_msgs::CameraInfo>(
        this->filtered_camera_name + "/camera_info", 1);
    this->ir_flag_pub = this->nh->advertise<std_msgs::Bool>(
        this->filtered_camera_name + "/is_ir", 1);

    this->image_sub = this->nh->subscribe(
            this->rgb_camera_name + this->raw_image_topic, 1, &FilteredCamera::rgb_callback, this);
    this->camera_info_sub = this->nh->subscribe(
        this->rgb_camera_name + "/camera_info", 1, &FilteredCamera::camera_info_callback, this);

    this->toggle_camera_sub = this->nh->subscribe(
        "brightness_analyser/toggle_camera", 1, &FilteredCamera::toggle_camera_callback, this);
}

void FilteredCamera::toggle_camera_callback(const std_msgs::Bool::ConstPtr& msg) {
    this->needs_change = true;
}

void FilteredCamera::ir_callback(const sensor_msgs::ImageConstPtr& img) {
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
}

void FilteredCamera::rgb_callback(const sensor_msgs::ImageConstPtr& img) {
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
}

void FilteredCamera::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info) {
    sensor_msgs::CameraInfo camera_info;
    this->camera_info_pub.publish(*info);
}

void FilteredCamera::toggle_camera_subscription() {
    std_msgs::Bool ir_msg;
    ir_msg.data = this->is_ir;
    this->ir_flag_pub.publish(ir_msg);
    
    if (this->is_ir && this->needs_change) {
        this->image_sub.shutdown();
        this->camera_info_sub.shutdown();

        this->image_sub = this->nh->subscribe(
            this->rgb_camera_name + this->raw_image_topic, 1, &FilteredCamera::rgb_callback, this);
        this->camera_info_sub = this->nh->subscribe(
            this->rgb_camera_name + "/camera_info", 1, &FilteredCamera::camera_info_callback, this);

        this->is_ir = false;
        this->needs_change = false;
    } else if (!this->is_ir && this->needs_change) {
        this->image_sub.shutdown();
        this->camera_info_sub.shutdown();

        this->image_sub = this->nh->subscribe(
            this->ir_camera_name + this->raw_image_topic, 1, &FilteredCamera::ir_callback, this);
        this->camera_info_sub = this->nh->subscribe(
            this->ir_camera_name + "/camera_info", 1, &FilteredCamera::camera_info_callback, this);

        this->is_ir = true;
        this->needs_change = false;
    }
}
#include "rov_cameras/mjpeg_camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

MJPEG_Camera::MJPEG_Camera(std::string device, uint8_t pub_id) : rclcpp::Node(std::string("camera_mjpeg") + std::to_string(static_cast<int>(pub_id))) {
    image_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(std::string("camera_mjpeg") + std::to_string(static_cast<int>(pub_id)) + std::string("/image_raw/compressed"), 10);
    poll_func = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&MJPEG_Camera::poll, this));

    char pipeline[1025] = {0};
    snprintf(pipeline, sizeof(pipeline)-1, PIPELINE_MJPEG_F, device.c_str());
    cap = cv::VideoCapture(std::string(pipeline), cv::CAP_GSTREAMER);
    RCLCPP_INFO(this->get_logger(), "Device %s initialized as MJPEG Camera", device.c_str());
}

void MJPEG_Camera::poll() {
    cap.grab();
    cap.retrieve(this->img_i420);
    cv::cvtColor(this->img_i420, this->img_bgr, cv::COLOR_YUV2BGR_I420, 3);
    sensor_msgs::msg::CompressedImage msg;
    std_msgs::msg::Header header;
    header.set__stamp(this->now());
    cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, this->img_bgr);
    img_bridge.toCompressedImageMsg(msg, cv_bridge::Format::PNG);
    image_pub->publish(msg);
}

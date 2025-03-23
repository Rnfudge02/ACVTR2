//Copyright [2025] [Robert Fudge]
//SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
//SPDX-License-Identifier: Apache-2.0
#pragma once

//ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


//Standard library includes
#include <vector>
#include <string>
#include <memory>

namespace acvtr2 {

    class ImageToVideoConverter : public rclcpp::Node {
    public:
        ImageToVideoConverter();
        ~ImageToVideoConverter();

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        void process_frame(const cv::Mat& frame);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
        std::unique_ptr<cv::VideoWriter> video_writer_;
        cv::Size frame_size_;
        std::string output_file_;
        int fps_;
        bool compressed_;
    };
}
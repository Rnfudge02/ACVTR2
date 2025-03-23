#include "acvtr2_base/ImageToVideoConverter.hpp"

using namespace acvtr2;

ImageToVideoConverter::ImageToVideoConverter() : Node("image_to_video_converter") {
    //Declare parameters
    this->declare_parameter<std::string>("topic", "/camera/camera/color/image_raw");
    this->declare_parameter<std::string>("output", "output.mp4");
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<bool>("compressed", false);
    
    //Retrieve parameters
    std::string topic_name = this->get_parameter("topic").as_string();
    output_file_ = this->get_parameter("output").as_string();
    fps_ = this->get_parameter("fps").as_int();
    compressed_ = this->get_parameter("compressed").as_bool();
    
    if (topic_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Must specify input topic!");
        throw std::runtime_error("Input topic not specified");
    }
    
    //Initialize video writer
    frame_size_ = cv::Size(0, 0);
    
    // Create appropriate subscriber
    if (compressed_) {
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic_name, 10,
        std::bind(&ImageToVideoConverter::compressed_callback, this, std::placeholders::_1));
    }
    
    else {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10,
        std::bind(&ImageToVideoConverter::image_callback, this, std::placeholders::_1));
    }
    
    RCLCPP_INFO(this->get_logger(), "Listening to %s, saving to %s", 
    topic_name.c_str(), output_file_.c_str());
}

ImageToVideoConverter::~ImageToVideoConverter() {
    if (video_writer_ && video_writer_->isOpened()) {
      video_writer_->release();
      RCLCPP_INFO(this->get_logger(), "Video writer released");
    }
  }

void ImageToVideoConverter::compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // Create buffer matrix for compressed data
        cv::Mat buffer(1, msg->data.size(), CV_8UC1, (void*)msg->data.data());
        cv::Mat cv_image = cv::imdecode(buffer, cv::IMREAD_COLOR);
    
        if (cv_image.empty()) {
            throw std::runtime_error("Failed to decompress image");
        }
        
        process_frame(cv_image);
    }

    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Compressed image error: %s", e.what());
    }
}

void ImageToVideoConverter::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image: encoding = %s, width = %d, height = %d", msg->encoding.c_str(), msg->width, msg->height);

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        process_frame(cv_ptr->image);
    }

    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image conversion error: %s", e.what());
    }
}

void ImageToVideoConverter::initialize_writer(const cv::Mat& frame) {
    frame_size_ = cv::Size(frame.cols, frame.rows);
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    video_writer_ = std::make_unique<cv::VideoWriter>(output_file_, fourcc, fps_, frame_size_);
    
    if (!video_writer_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize video writer");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initialized video writer with %dx%d @ %d FPS", frame_size_.width, frame_size_.height, fps_);
}

void ImageToVideoConverter::process_frame(const cv::Mat& frame) {
    if (!video_writer_ || !video_writer_->isOpened()) {
        initialize_writer(frame);
    }

    if (video_writer_->isOpened()) {
        video_writer_->write(frame);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ImageToVideoConverter>();
        rclcpp::spin(node);
    }
    
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("image_to_video_converter"), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
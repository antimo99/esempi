#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthReader : public rclcpp::Node {
public:
    DepthReader() : Node("depth_reader") {
        // Subscribe to the depth image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_registered/image_rect",
            10,
            std::bind(&DepthReader::depthCallback, this, std::placeholders::_1)
        );
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_16UC1);

            // Specify the pixel coordinates
            int x = 320; // Example X coordinate
            int y = 240; // Example Y coordinate

            // Check bounds
            if (x >= 0 && x < cv_ptr->image.cols && y >= 0 && y < cv_ptr->image.rows) {
                uint16_t depth_value = cv_ptr->image.at<uint16_t>(y, x);
                RCLCPP_INFO(this->get_logger(), "Depth value compressed at (%d, %d): %u", x, y, static_cast<int>depth_value.data); 
                RCLCPP_INFO(this->get_logger(), "Depth value in mm at (%d, %d): %u", x, y, (depth_value / 255.0) * 8000);
            } else {
                RCLCPP_WARN(this->get_logger(), "Pixel coordinates out of bounds!");
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthReader>());
    rclcpp::shutdown();
    return 0;
}

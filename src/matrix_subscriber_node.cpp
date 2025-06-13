#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>

class MatrixSubscriber : public rclcpp::Node {
public:
    MatrixSubscriber()
    : Node("matrix_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "matrix", 10,
            std::bind(&MatrixSubscriber::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        size_t rows = 2;
        size_t cols = 3;
        if (msg->data.size() != rows *cols) {
            RCLCPP_WARN(this->get_logger(), "Size mismatch! Got %zu elements.", msg->data.size());
            return;
        }
        std::vector<std::vector<double>> reshaped(rows,std::vector<double>(cols));
        for (size_t i=0; i<rows; ++i) {
            for (size_t j=0; j<cols; ++j) {
                reshaped[i][j] = msg->data[i*cols+j];
            }
            std::cout <<std::endl;
        }

        std::cout << "Resahped matrix: " << std::endl;
        for (const auto& row: reshaped) {
            for (double val : row) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MatrixSubscriber>());
    rclcpp::shutdown();
    return 0;
}

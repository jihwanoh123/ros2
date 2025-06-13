#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>

// Include your Matrix class definition here
// For now, we'll use a simple inline version
template<typename T>
class Matrix {
private:
    std::vector<std::vector<T>> data;
public:
    Matrix(std::initializer_list<std::vector<T>> rows) : data(rows) {}
    size_t rows() const { return data.size(); }
    size_t cols() const { return data[0].size(); }
    const std::vector<T>& operator[](size_t i) const { return data[i]; }

    std::vector<T> flatten() const {
        std::vector<T> flat;
        for (const auto& row : data) {
            flat.insert(flat.end(), row.begin(), row.end());
        }
        return flat;
    }
};

class MatrixPublisher : public rclcpp::Node {
public:
    MatrixPublisher() : Node("matrix_publisher") {
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("matrix", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MatrixPublisher::publish_matrix, this)
        );
    }

private:
    void publish_matrix() {
        Matrix<double> mat = {
            {1.0, 2.0, 3.0},
            {4.0, 5.0, 6.0}
        };

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = mat.flatten();

        RCLCPP_INFO(this->get_logger(), "Publishing matrix with %ld elements", msg.data.size());
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
    
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MatrixPublisher>());
    rclcpp::shutdown();
    return 0;
}

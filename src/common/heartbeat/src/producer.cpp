#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>

class Producer : public rclcpp::Node {
public:
    Producer() : Node("heartbeat_producer") {
        using namespace std::chrono_literals;
        heartbeat_publisher = this->create_publisher<std_msgs::msg::Bool>("heartbeat", 10);
        heartbeat_timer = this->create_wall_timer(1s, std::bind(&Producer::heartbeat, this));
    }
private:
    void heartbeat() {
        std::cout << "Beat" << std::endl;
        heartbeat_publisher->publish(std_msgs::msg::Bool());
    }

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_publisher;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Producer>());

    rclcpp::shutdown();
}
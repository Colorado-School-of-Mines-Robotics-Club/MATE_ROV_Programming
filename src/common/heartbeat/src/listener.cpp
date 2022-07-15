#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>

class Listener : public rclcpp::Node {
public:
    Listener() : Node("heartbeat_lsitener") {
        using namespace std::chrono_literals;
        heartbeat_subscription = this->create_subscription<std_msgs::msg::Bool>("heartbeat", 10, std::bind(&Listener::heartbeat, this, std::placeholders::_1));
    }
private:
    void heartbeat(std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::cout << "Beat" << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_subscription;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Listener>());

    rclcpp::shutdown();
}
#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/manipulator_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_servo.hpp"

#define NUM_SERVOS 5

enum ServoMap {
    ELBOW_1 = 8,
    ELBOW_2 = 9,
    LEVEL = 10,
    WRIST = 11,
    CLAMP = 12,
};

class ManipulatorControl : public rclcpp::Node {
public:
    ManipulatorControl() : Node(std::string("manipulator_control")) {
        // create servos
        registerServos();

        _subscription = this->create_subscription<rov_interfaces::msg::ManipulatorSetpoints>(
            "manipulator_setpoints", 10, std::bind(&ManipulatorControl::setpoint_callback, this, std::placeholders::_1));
        _publisher = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);
    }

private:
    void registerServos() {
       // create service client
        auto pca9685 = this->create_client<rov_interfaces::srv::CreateServo>("create_servo");
        for(int i=8; i < NUM_SERVOS; i++) {
            // create continuous servo creation requests on channels 8 -> NUM_SERVOS
            auto req = std::make_shared<rov_interfaces::srv::CreateServo_Request>();
            req->channel = i;
            // ensure service is not busy
            while(!pca9685->wait_for_service(std::chrono::milliseconds(100))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // create lambda function to handle asynchronous callbacks
            using ServiceResponseFuture = rclcpp::Client<rov_interfaces::srv::CreateServo>::SharedFuture;
            auto registerThrusterCallback = [this](ServiceResponseFuture future) {
                auto res = future.get();
                if(res->result)
                    RCLCPP_INFO(this->get_logger(), "Successfully registered continuous servo on channel %i", res->channel);
                else
                    RCLCPP_ERROR(this->get_logger(), "Unsuccessfully registered continuous servo on channel %i", res->channel);
            };
            // asynchronously send these servo creation requests
            auto future_result = pca9685->async_send_request(req, registerThrusterCallback);
        }
    }

    void setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg) const {
        std::array<rov_interfaces::msg::PWM, NUM_SERVOS> messages;
        messages[0].channel = ServoMap::ELBOW_1;
        messages[1].channel = ServoMap::ELBOW_2;
        messages[2].channel = ServoMap::LEVEL;
        messages[3].channel = ServoMap::WRIST;
        messages[4].channel = ServoMap::CLAMP;

        messages[0].angle_or_throttle = msg->elbow;
        messages[1].angle_or_throttle = 180-msg->elbow;
        messages[2].angle_or_throttle = msg->level;
        messages[3].angle_or_throttle = msg->wrist;
        messages[3].angle_or_throttle = msg->clamp;

        for (auto& msg : messages) {
            _publisher->publish(msg);
        }
    }

    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;
    rclcpp::Subscription<rov_interfaces::msg::ManipulatorSetpoints>::SharedPtr _subscription;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorControl>());
    rclcpp::shutdown();
    return 0;
}
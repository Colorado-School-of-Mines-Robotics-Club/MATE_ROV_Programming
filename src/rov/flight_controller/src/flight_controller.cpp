#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rov_interfaces/msg/bno055_data.hpp"
#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"

#include "flight_controller/Thruster.hpp"

#define NUM_THRUSTERS 8

#define MIN_THRUST_VALUE -28.44
#define MAX_THRUST_VALUE 36.4
#define MIN_THROTTLE_CUTOFF -0.3
#define MAX_THROTTLE_CUTOFF 0.4

class FlightController : public rclcpp::Node {
public:
    FlightController() : Node(std::string("flight_controller")) {
        // define thrusters TODO: replace with a config file? (temp values atm)
        float x = sqrt(2)/2;
        thrusters[0] = Thruster(Eigen::Vector3d(1,    1,    0),  Eigen::Vector3d( 0,  0,  1), 0);
        thrusters[1] = Thruster(Eigen::Vector3d(1,   -1,    0),  Eigen::Vector3d( 0,  0,  1), 1);
        thrusters[2] = Thruster(Eigen::Vector3d(1,   -0.5,  0),  Eigen::Vector3d( x,  x,  0), 2);
        thrusters[3] = Thruster(Eigen::Vector3d(1,    0.5,  0),  Eigen::Vector3d( x, -x,  0), 3);
        thrusters[4] = Thruster(Eigen::Vector3d(-1,   1,    0),  Eigen::Vector3d( 0,  0,  1), 12);
        thrusters[5] = Thruster(Eigen::Vector3d(-1,  -1,    0),  Eigen::Vector3d( 0,  0,  1), 13);
        thrusters[6] = Thruster(Eigen::Vector3d(-1,  -0.5,  0),  Eigen::Vector3d(-x,  x,  0), 14);
        thrusters[7] = Thruster(Eigen::Vector3d(-1,   0.5,  0),  Eigen::Vector3d(-x, -x,  0), 15);

        std::array<Eigen::VectorXd, NUM_THRUSTERS> temp;
        for(int i = 0; i < NUM_THRUSTERS; i++) {
            Thruster t = thrusters[i];
            // calculate linear and rotation contribution
            Eigen::Vector3d linear_contribution(t.thrust * MAX_THRUST_VALUE);
            Eigen::Vector3d rotation_contribution(t.position.cross(t.thrust * MAX_THRUST_VALUE));
            this->thruster_index_to_PWM_pin.emplace(std::make_pair(i, t.pwm_pin));

            // concatenate them
            temp[i] = Eigen::VectorXd(6);
            temp[i] << linear_contribution, rotation_contribution;
            thruster_geometry.col(i) << temp[i];
        }
        
        // compute the pseudo inverse of the thruster geometry
        this->thruster_geometry_pseudo_inverse = this->thruster_geometry.transpose() * (this->thruster_geometry*this->thruster_geometry.transpose()).inverse();
        this->thruster_coefficient_matrix.diagonal() << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0; // thrusters are identical T200s running at ~12V
        this->thruster_coefficient_matrix_times_geometry = this->thruster_coefficient_matrix * this->thruster_geometry_pseudo_inverse;

        // use PWM service to register thrusters on PCA9685
        this->registerThrusters();
        
        // create ros subscriptions and publishers
        thruster_setpoint_subscription = this->create_subscription<rov_interfaces::msg::ThrusterSetpoints>("thruster_setpoints", 10, std::bind(&FlightController::setpoint_callback, this, std::placeholders::_1));
        bno_data_subscription = this->create_subscription<rov_interfaces::msg::BNO055Data>("bno055_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bno_callback, this, std::placeholders::_1));
        
        _publisher = this->create_publisher<rov_interfaces::msg::PWM>("pwm", 10);

        // about 60 hz update rate
#if(USE_PID == true)
        pid_control_loop = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&FlightController::update, this));
#else
        pid_control_loop = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&FlightController::updateSimple, this));
#endif
    }
private:
    void registerThrusters() {
        // create service client
        auto pca9685 = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo");
        std::array<std::shared_future<std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Response>>, NUM_THRUSTERS> requests;
        for(int i=0; i < NUM_THRUSTERS; i++) {
            // create continuous servo creation requests on channels 0 -> NUM_THRUSTERS
            auto req = std::make_shared<rov_interfaces::srv::CreateContinuousServo_Request>();
            req->channel = i;
            // ensure service is not busy
            while(!pca9685->wait_for_service(std::chrono::milliseconds(100))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // asynchronously send these servo creation requests
            requests[i] = pca9685->async_send_request(req);
        }

        // wait for requests to be completed
        for(int i=0; i < NUM_THRUSTERS; i++) {
            auto status = requests[i].wait_for(std::chrono::milliseconds(100));
            if(status == std::future_status::ready) {
                auto res = requests[i].get();
                if(res->result) {
                    RCLCPP_INFO(this->get_logger(), "Successfully registered continuous servo on channel %i", res->channel);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Unsuccessfully registered continuous servo on channel %i", res->channel);
                }
            } else {
                i--;
                continue;
            }
        }
    }

    void setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr setpoints) {
        std::lock_guard<std::mutex>(this->setpoint_mutex);
        std::lock_guard<std::mutex>(this->stall_mutex);
        translation_setpoints(0,0) = setpoints->vx;
        translation_setpoints(1,0) = setpoints->vy;
        translation_setpoints(2,0) = setpoints->vz;
        attitude_setpoints(0,0) = setpoints->omegax;
        attitude_setpoints(1,0) = setpoints->omegay;
        attitude_setpoints(2,0) = setpoints->omegaz;
    }

    void bno_callback(const rov_interfaces::msg::BNO055Data::SharedPtr bno_data) {
        std::lock_guard<std::mutex>(this->bno_mutex);
        this->bno_data = *bno_data.get();
    }

    void updateSimple() {
        // this is direct mapping from velocities to output force
        // warning: no proportional controller; might be unweildy to use
        Eigen::Matrix<double, 6, 1> desired_throttles;
        desired_throttles << translation_setpoints, attitude_setpoints;

        // scale thrusters to account for large thrust demands on a single thruster
        // TODO: implement
        Eigen::Matrix<double, NUM_THRUSTERS, 1> throttles;

        // publish PWM values
        for(int i = 0; i < NUM_THRUSTERS; i++) {
            rov_interfaces::msg::PWM msg;
            msg.angle_or_throttle = static_cast<float>(throttles(i,0)); // this is a source of noise in output signals, may cause system instability??
            msg.channel = thruster_index_to_PWM_pin.at(i);
            _publisher->publish(msg);
        }
    }

    void update() {
        Eigen::Vector3d desired_force;
        Eigen::Vector3d desired_torque;

        // update dt
        auto now = std::chrono::high_resolution_clock::now();
        int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_updated).count();
        this->last_updated = now;

        // fill the matrixes based on bno and setpoint data
        this->setpoint_mutex.lock();
        this->bno_mutex.lock();
        Eigen::Vector3d linear_accel(bno_data.linearaccel.i, bno_data.linearaccel.j,bno_data.linearaccel.k); // m/s^2
        Eigen::Vector3d linear_velocity = linear_velocity + linear_accel * dt_ms / 1000; // get an approximation of linear velocity
        linear_velocity_err_last = linear_velocity_err;
        linear_velocity_err[0] = translation_setpoints(0,0) - linear_velocity[0];
        linear_velocity_err[1] = translation_setpoints(1,0) - linear_velocity[1];
        linear_velocity_err[2] = translation_setpoints(2,0) - linear_velocity[2];
        Eigen::Vector3d ha = dt_ms * 0.5 * attitude_setpoints; // vector of half angle
        Eigen::Vector3d omega = Eigen::Vector3d(bno_data.gyroscope.i, bno_data.gyroscope.j, bno_data.gyroscope.k);
        auto quaternion_measured = Eigen::Quaterniond(bno_data.orientation.w, bno_data.orientation.i, bno_data.orientation.j, bno_data.orientation.k);
        this->setpoint_mutex.unlock();
        this->bno_mutex.unlock();

        // update desired force
        // Linear, discrete-time PID controller
        // lambdas for linear PID loop
        auto P = [](double& kp, Eigen::Vector3d& linear_velocity_err)->Eigen::Vector3d {
            return kp * linear_velocity_err;
        };
        auto I = [](double& ki, Eigen::Vector3d& linear_velocity_err, int& dt_ms)->Eigen::Vector3d {
            return ki * linear_velocity_err * dt_ms / 1000;
        };
        auto D = [](double& kd, Eigen::Vector3d& linear_velocity_err, Eigen::Vector3d& linear_velocity_err_last, int& dt_ms)->Eigen::Vector3d {
            return kd* (linear_velocity_err - linear_velocity_err_last) / (static_cast<double>(dt_ms) / 1000);
        };

        // P + I + D
        desired_force = P(kp,linear_velocity_err)
                        + (linear_integral += I(ki, linear_velocity_err, dt_ms))
                        + D(kd, linear_velocity_err, linear_velocity_err_last, dt_ms);

        // update desired torque
        // TODO: look at this if performance needs to be improved (loses accuracy tho)
        // see https://stackoverflow.com/questions/24197182/efficient-quaternion-angular-velocity/24201879#24201879
        double l = ha.norm(); // magnitude
        if (l > 0) {
            ha *= sin(l) / l;
            quaternion_reference = Eigen::Quaterniond(cos(l), ha.x(), ha.y(), ha.z());
        } else {
            quaternion_reference = Eigen::Quaterniond(1.0, ha.x(), ha.y(), ha.z());
        }
        // desired orientation is only known by the pilot, quaternion reference defines a reference from a unity quaternion (1,0,0,0)
        // to get actual reference, this must be multiplied with the measured quaternion to get desired orientation
        // TODO: ensure that when w = 0, this is close to, or equal to unity
        quaternion_reference = quaternion_reference * quaternion_measured;

        // Non Linear P^2 Quaternion based control scheme
        // For derivation of controller see: http://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf
        // For derivation of dynamics and control allocation see: https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf
        auto q_err = quaternion_reference * quaternion_measured.conjugate(); // hamilton product (hopefully)
        Eigen::Vector3d axis_err;

        if(q_err.w() < 0) { // this could be simplified to a negation but eh
            axis_err = q_err.conjugate().vec();
        } else {
            axis_err = q_err.vec();
        }

        desired_torque = (-Pq * axis_err) - (Pw * omega);

        // control allocation
        // u = K^-1 * T^+ * t
        // 8x1 = 8x8 * 8x6 * 6x1 :)
        Eigen::Matrix<double, 6, 1> forcesAndTorques;
        forcesAndTorques << desired_force, desired_torque;

        // solve Ax = b and normalize thrust such that it satisfies MIN_THRUST_VALUE <= throttles[j] <= MAX_THRUST_VALUE 
        // while scaling thrusters to account for large thrust demands on a single thruster
        Eigen::Matrix<double, NUM_THRUSTERS, 1> throttles = thrust2throttle((this->thruster_coefficient_matrix_times_geometry) * forcesAndTorques);

        // publish PWM values
        for(int i = 0; i < NUM_THRUSTERS; i++) {
            rov_interfaces::msg::PWM msg;
            msg.angle_or_throttle = static_cast<float>(throttles(i,0)); // this is a source of noise in output signals, may cause system instability??
            msg.channel = thruster_index_to_PWM_pin.at(i);
            _publisher->publish(msg);
        }
    }

    Eigen::Matrix<double,NUM_THRUSTERS,1> thrust2throttle(Eigen::Matrix<double, NUM_THRUSTERS, 1> thrust) {
        Eigen::Index loc;
        thrust.minCoeff(&loc);
        double ratioA = std::abs(MIN_THRUST_VALUE / std::min(thrust(loc), MIN_THRUST_VALUE));
        thrust.maxCoeff(&loc);
        double ratioB = std::abs(MAX_THRUST_VALUE / std::max(thrust(loc), MAX_THRUST_VALUE));
        if(ratioA > ratioB) {
            thrust = thrust * ratioA;
        } else {
            thrust = thrust * ratioB;
        }

        // inverse thrust function
        Eigen::Matrix<double,NUM_THRUSTERS,1> toret;
        for(int i = 0; i < NUM_THRUSTERS; i++) {
            if(thrust(i,0) < MIN_THROTTLE_CUTOFF) {
                // see documentation to understand origin of this equation
                toret(i,0) = std::max(-0.0991 + 0.0505 * thrust(i,0) + 1.22e-3 * pow(thrust(i,0),2) + 1.91e-5 * pow(thrust(i,0),3),-1.0);
            } else if (toret(i,0) > MAX_THROTTLE_CUTOFF) {
                // see documentation to understand origin of this equation
                toret(i,0) = std::min(0.0986 + 0.0408 * thrust(i,0) + -8.14e-4 * pow(thrust(i,0),2) + 1.01e-5 * pow(thrust(i,0),3),1.0);
            } else {
                toret(i,0) = 0;
            }
        }

        return toret;
    }

    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr thruster_setpoint_subscription;
    rclcpp::Subscription<rov_interfaces::msg::BNO055Data>::SharedPtr bno_data_subscription;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;

    rclcpp::TimerBase::SharedPtr pid_control_loop;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_updated;

    std::mutex stall_mutex;

    rov_interfaces::msg::BNO055Data bno_data;
    std::mutex bno_mutex;
    Eigen::Vector3d translation_setpoints = Eigen::Vector3d(3,1);
    Eigen::Vector3d attitude_setpoints = Eigen::Vector3d(3,1);
    std::mutex setpoint_mutex;
    Eigen::Quaterniond quaternion_reference;
    Eigen::Vector3d linear_accel_last;
    Eigen::Vector3d linear_integral;
    Eigen::Vector3d linear_velocity_err = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d linear_velocity_err_last;

    std::array<Thruster, NUM_THRUSTERS> thrusters;
    Eigen::Matrix<double, 6, NUM_THRUSTERS> thruster_geometry;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_geometry_pseudo_inverse;
    Eigen::DiagonalMatrix<double, NUM_THRUSTERS> thruster_coefficient_matrix;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_coefficient_matrix_times_geometry;
    std::unordered_map<int, int> thruster_index_to_PWM_pin;

    double Pq = 1.0, Pw = 1.0; // TODO: tune these gain constants
    double kp = 1.0, ki = 1.0, kd = 1.0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightController>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
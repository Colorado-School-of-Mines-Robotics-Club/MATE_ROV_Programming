#include <assert.h>
#include <chrono>
#include <csignal>
#include <glob.h>
#include <iostream>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <signal.h>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <compressed_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Testing Pipeline
const std::string pipeline = std::string("v4l2src device=/dev/video0 io-mode=2 ! video/x-raw,format=YUY2,width=320,height=240,framerate=30/1 ! videoconvert ! appsink");

// Release Pipeline
// IF YOU TOUCH THIS I WILL KILL YOU
// const std::string pipeline = std::string("v4l2src device=%s io-mode=2 ! image/jpeg,framerate=30/1,width=320,height=240 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nvvidconv ! video/x-raw,format=RGBA ! videoconvert ! video/x-raw,format=BGR ! videoconvert ! appsink");


template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}


class ROV_Cameras : public rclcpp::Node {
public:
    ROV_Cameras() : Node(std::string("rov_cameras")) {
        this->collect_cameras();

        // create camera image publishers
        this->image_publishers.emplace(std::make_pair(0, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam0_image",10)));
        this->image_publishers.emplace(std::make_pair(1, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam1_image",10)));
        this->image_publishers.emplace(std::make_pair(2, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam2_image",10)));
        this->image_publishers.emplace(std::make_pair(3, this->create_publisher<sensor_msgs::msg::CompressedImage>("cam3_image",10)));

        // create camera control subscribers
        std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback0 = std::bind(&ROV_Cameras::camera_control, this, 0, std::placeholders::_1);
        camera_control0 = this->create_subscription<std_msgs::msg::Bool>("cam0_control", 10, callback0);
        std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback1 = std::bind(&ROV_Cameras::camera_control, this, 1, std::placeholders::_1);
        camera_control1 = this->create_subscription<std_msgs::msg::Bool>("cam1_control", 10, callback1);
        std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback2 = std::bind(&ROV_Cameras::camera_control, this, 2, std::placeholders::_1);
        camera_control2 = this->create_subscription<std_msgs::msg::Bool>("cam2_control", 10, callback2);
        std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback3 = std::bind(&ROV_Cameras::camera_control, this, 3, std::placeholders::_1);
        camera_control3 = this->create_subscription<std_msgs::msg::Bool>("cam3_control", 10, callback3);

        // Create worker threads
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 0, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 1, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 2, std::reference_wrapper<bool>(this->running))));
        this->workers.push_back(std::thread(std::bind(&ROV_Cameras::camera_callback, this, 3, std::reference_wrapper<bool>(this->running))));

        // ensure this ROS node does not exit before destructor is called, do nothing every 100ms :)
        do_nothing_timer = this->create_wall_timer(100ms, std::bind(&ROV_Cameras::do_nothing, this));
    }

    ~ROV_Cameras() {
        this->running = false;
        usleep(40*1000); 
        // ensure threads are shutdown, join before destruction
        for(std::size_t i = 0; i < this->workers.size(); i++) {
            workers[i].join();
        }
        for(std::size_t i = 0; i < this->cameras.size(); i++) {
            cameras[i]->release(); // explicitly release them (destructor works aswell but doesnt hurt to be safe)
        }
    }

    void sigint() {
        this->running = false;
        usleep(40*1000); 
        // ensure threads are shutdown, join before destruction
        for(std::size_t i = 0; i < this->workers.size(); i++) {
            workers[i].join();
        }
        for(std::size_t i = 0; i < this->cameras.size(); i++) {
            cameras[i]->release(); // explicitly release them (destructor works aswell but doesnt hurt to be safe)
        }
    }

private:
    void do_nothing() {

    }

    void camera_control(int cam, const std_msgs::msg::Bool::SharedPtr state) {
        std::cout << "camera control callback" << std::endl;
        std::lock_guard<std::mutex>(this->should_camera_run_mutex[cam]);
        this->should_camera_run[cam] = state->data;
    }

    void camera_callback(int camera, std::reference_wrapper<bool> shouldBeRunning) {
        // ensure this index is valid
        if(this->cameras.size()-1 < static_cast<std::size_t>(camera)) {
            std::cout << "camera" << camera << " is not created. Destroying this thread" << std::endl;
            return;
        }
        if(!this->cameras[camera]->isOpened()) {
            std::cout << "camera not opened" << std::endl;
            std::cout << "camera" << camera << " was created, but is not opened. Destroying this thread" << std::endl;
            return;
        }
        // create single buffer for images (is copied later)
        // TODO: look at replacing CvImage bridge if performance is required, it makes extraneous copies that we can reduce
        cv::Mat image(240, 320, CV_8UC3);
        // should the camera THREAD be running?
        while(shouldBeRunning.get()) {
            usleep(33 * 1000); // sleep for at least 33 milliseconds (~30fps)
            // should camera SENDING be running?
            this->should_camera_run_mutex[camera].lock();
            if(!this->should_camera_run[camera]) {
                this->should_camera_run_mutex[camera].unlock();
                continue;
            } else {
                this->should_camera_run_mutex[camera].unlock();
            }
            // get frame
            if(this->cameras[camera]->read(image)) {
                // to locally test cameras uncomment these lines
                // cv::imshow("test", image);
                // cv::waitKey(1);
                sensor_msgs::msg::CompressedImage msg;
                std_msgs::msg::Header header = std_msgs::msg::Header();
                header.set__stamp(this->now());
                cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
                img_bridge.toCompressedImageMsg(msg);
                this->image_publishers[camera]->publish(msg);
            } else { // this could probably be put into a function since its 99% repeated code, but whateva
                // camera didnt grab frame???
                RCLCPP_DEBUG_ONCE(this->get_logger(), "Lost connection to Camera %i", camera);
                this->cameras[camera]->release();
                usleep(100*1000); // sleep for 100 milliseconds

                // collect the video devices
                glob_t glob_result;
                memset(&glob_result, 0, sizeof(glob_result));
                int ret_val = glob("/dev/video?", GLOB_TILDE, NULL, &glob_result);

                // error handling
                if(ret_val != 0) {
                    globfree(&glob_result);
                    continue;
                }

                // collect filenames of cameras in /dev/
                std::vector<std::string> filenames;
                for(size_t i = 0; i < glob_result.gl_pathc; i++) {
                    filenames.push_back(std::string(glob_result.gl_pathv[i]));
                }

                // glob cleanup
                globfree(&glob_result);

                // test filenames to see if camera is available
                for(std::string camera_path : filenames) {
                    std::shared_ptr<cv::VideoCapture> camera_device = std::make_shared<cv::VideoCapture>(string_format(pipeline, camera_path.c_str()), cv::CAP_GSTREAMER);
                    usleep(1000 * 1000); // ensure camera is captured and opened
                    if(!camera_device->isOpened()) {
                        RCLCPP_DEBUG(this->get_logger(), "During reconnection attempt of Camera %i, %s is not a camera or could not be opened (might already be in use).", camera, camera_path);
                    } else {
                        camera_device->set(cv::CAP_PROP_FRAME_WIDTH,320);
                        camera_device->set(cv::CAP_PROP_FRAME_HEIGHT,240);
                        RCLCPP_DEBUG(this->get_logger(), "During reconnection attempt of Camera %i, %s successfully opened as video capture object", camera, camera_path);
                        this->cameras[camera] = camera_device;
                    }
                }
            }
        }
    }

    void collect_cameras() {
        // collect cameras from /dev/video? with glob
        glob_t glob_result;
        memset(&glob_result, 0, sizeof(glob_result));
        int ret_val = glob("/dev/video?", GLOB_TILDE, NULL, &glob_result);

        // error handling
        if(ret_val != 0) {
            globfree(&glob_result);
            std::stringstream ss;
            ss << "glob failed with return value " << ret_val << std::endl;
            throw std::runtime_error(ss.str());
        }

        // collect filenames of cameras in /dev/
        std::vector<std::string> filenames;
        for(size_t i = 0; i < glob_result.gl_pathc; i++) {
            filenames.push_back(std::string(glob_result.gl_pathv[i]));
        }

        // glob cleanup
        globfree(&glob_result);

        // test filenames to see if camera is available
        for(std::string camera_path : filenames) {
            std::cout << "Evaluating pipeline: " << string_format(pipeline, camera_path.c_str()) << std::endl;
            std::shared_ptr<cv::VideoCapture> camera = std::make_shared<cv::VideoCapture>(string_format(pipeline, camera_path.c_str()), cv::CAP_GSTREAMER);
            usleep(1000 * 1000); // ensure camera is captured and opened
            if(!camera->isOpened()) {
                RCLCPP_DEBUG(this->get_logger(), "%s is not a camera or could not be opened.", camera_path);
                std::cout << camera_path << " is not a camera or could not be opened." << std::endl;
            } else {
                camera->set(cv::CAP_PROP_FRAME_WIDTH,320);
                camera->set(cv::CAP_PROP_FRAME_HEIGHT,240);
                RCLCPP_DEBUG(this->get_logger(), "%s successfully opened as video capture object", camera_path);
                std::cout << camera_path << " successfully opened as video capture object" << std::endl;
                this->cameras.push_back(camera);
            }
        }
        // ensure we don't have too many cameras (allow less than 4)
        assert(this->cameras.size() <= 4);
    }

    std::vector<std::thread> workers;
    std::vector<std::shared_ptr<cv::VideoCapture>> cameras;
    std::array<bool, 4> should_camera_run = {true, true, true, true};
    std::array<std::mutex, 4> should_camera_run_mutex;
    std::unordered_map<int, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>> image_publishers;
    bool running = true;

    rclcpp::TimerBase::SharedPtr do_nothing_timer;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_control0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_control1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_control2;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_control3;
};

namespace {
    std::function<void(int)> sigint_handler;
    void signal_handler(int signal) {
        sigint_handler(signal);
    }
}

int main(int argc, char ** argv) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROV_Cameras>();
    std::signal(SIGINT, signal_handler);
    sigint_handler = [node](int signal){std::cout << "received signal " << signal << std::endl; node->sigint(); rclcpp::shutdown(); exit(signal);};
    rclcpp::spin(node);
    rclcpp::shutdown();
}

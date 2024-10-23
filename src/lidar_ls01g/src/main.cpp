#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "uart_driver.h"

using namespace std::chrono_literals;

bool is_scan_stop = false;
bool is_motor_stop = false;
bool zero_as_max = true;
bool min_as_zero = true;
bool inverted = true;
std::string laser_link = "laser_link";
double angle_disable_min = -1;
double angle_disable_max = -1;
io_driver driver;

class LidarNode : public rclcpp::Node {
public:
    LidarNode() : Node("ls01g_node") {
        this->declare_parameter<std::string>("serial_port", "/dev/laser");
        this->declare_parameter<std::string>("laser_link", "laser_link");
        this->declare_parameter<double>("angle_disable_min", -1);
        this->declare_parameter<double>("angle_disable_max", -1);
        this->declare_parameter<bool>("zero_as_max", false);
        this->declare_parameter<bool>("min_as_zero", false);
        this->declare_parameter<bool>("inverted", false);

        this->get_parameter("serial_port", port_);
        this->get_parameter("laser_link", laser_link);
        this->get_parameter("angle_disable_min", angle_disable_min);
        this->get_parameter("angle_disable_max", angle_disable_max);
        this->get_parameter("zero_as_max", zero_as_max);
        this->get_parameter("min_as_zero", min_as_zero);
        this->get_parameter("inverted", inverted);

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        stop_sub_ = this->create_subscription<std_msgs::msg::Int32>("startOrStop", 10, std::bind(&LidarNode::startStopCB, this, std::placeholders::_1));

        int ret = driver.OpenSerial(port_.c_str(), B230400);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open port: %s", port_.c_str());
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Opened port: %s", port_.c_str());
        }

        if (inverted) {
            RCLCPP_INFO(this->get_logger(), "This laser is inverted, zero degree direction is align with line");
        }

        driver.StartScan();
        timer_ = this->create_wall_timer(100ms, std::bind(&LidarNode::read_and_publish, this));
    }

    void stop_motor() {
        if (!is_motor_stop) {
            driver.StopScan(STOP_MOTOR);
            is_motor_stop = true;
            RCLCPP_INFO(this->get_logger(), "Motor stopped");
        }
    }

private:
    void publish_scan(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub, double *dist, double *intensities, int count, rclcpp::Time start, double scan_time) {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = laser_link;
        scan_count++;
        scan_msg->angle_min = 0.0;
        scan_msg->angle_max = 2 * M_PI;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(count - 1);
        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(count - 1);

        scan_msg->range_min = 0.1;
        scan_msg->range_max = 10.0;

        scan_msg->intensities.resize(count);
        scan_msg->ranges.resize(count);

        if (!inverted) {
            for (int i = count - 1; i >= 0; i--) {
                if ((i >= angle_disable_min) && (i < angle_disable_max)) {
                    if (min_as_zero)
                        scan_msg->ranges[i] = 0.0;
                    else
                        scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                } else if (dist[count - i - 1] == 0.0 && zero_as_max)
                    scan_msg->ranges[i] = scan_msg->range_max - 0.2;
                else if (dist[count - i - 1] == 0.0)
                    scan_msg->ranges[i] = min_as_zero ? 0.0 : std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = dist[count - i - 1] / 1000.0;
                scan_msg->intensities[i] = floor(intensities[count - i - 1]);
            }
        } else {
            for (int i = 0; i <= 179; i++) {
                if ((i >= angle_disable_min) && (i < angle_disable_max)) {
                    scan_msg->ranges[i] = min_as_zero ? 0.0 : std::numeric_limits<float>::infinity();
                } else if (dist[179 - i] == 0.0 && zero_as_max)
                    scan_msg->ranges[i] = scan_msg->range_max - 0.2;
                else if (dist[179 - i] == 0.0)
                    scan_msg->ranges[i] = min_as_zero ? 0.0 : std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = dist[179 - i] / 1000.0;
                scan_msg->intensities[i] = floor(intensities[179 - i]);
            }
            for (int i = 180; i < 360; i++) {
                if ((i >= angle_disable_min) && (i < angle_disable_max)) {
                    scan_msg->ranges[i] = min_as_zero ? 0.0 : std::numeric_limits<float>::infinity();
                } else if (dist[540 - i] == 0.0 && zero_as_max)
                    scan_msg->ranges[i] = scan_msg->range_max - 0.2;
                else if (dist[540 - i] == 0.0)
                    scan_msg->ranges[i] = min_as_zero ? 0.0 : std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = dist[540 - i] / 1000.0;
                scan_msg->intensities[i] = floor(intensities[540 - i]);
            }
        }
        pub->publish(*scan_msg);
    }

    void startStopCB(const std_msgs::msg::Int32::SharedPtr msg) {
        Command cmd = static_cast<Command>(msg->data);
        switch (cmd) {
            case STOP_DATA:
                if (!is_scan_stop) {
                    driver.StopScan(STOP_DATA);
                    is_scan_stop = true;
                    RCLCPP_INFO(this->get_logger(), "Stop scan");
                }
                break;
            case STOP_MOTOR:
                if (!is_scan_stop) {
                    driver.StopScan(STOP_DATA);
                    is_scan_stop = true;
                    RCLCPP_INFO(this->get_logger(), "Stop scan");
                }
                if (!is_motor_stop) {
                    driver.StopScan(STOP_MOTOR);
                    is_motor_stop = true;
                    RCLCPP_INFO(this->get_logger(), "Stop motor");
                }
                break;
            case START_MOTOR_AND_SCAN:
                if (is_scan_stop) {
                    RCLCPP_INFO(this->get_logger(), "Start scan");
                    int res = driver.StartScan();
                    RCLCPP_INFO(this->get_logger(), "Start: %d", res);
                    is_scan_stop = false;
                    is_motor_stop = false;
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: %d", cmd);
                break;
        }
    }

    void read_and_publish() {
        if (is_scan_stop) return;

        double angle[PACKLEN + 10];
        double distance[PACKLEN + 10];
        double data[PACKLEN + 10];
        double data_intensity[PACKLEN + 10];
        double speed;
        int count = 0;

        auto starts = this->now();
        auto ends = this->now();
        memset(data, 0, sizeof(data));
        int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
        for (int i = 0; i < ret; i++) {
            data[i] = distance[i];
            data_intensity[i] = angle[i];
        }
        ends = this->now();
        float scan_duration = (ends - starts).seconds() * 1e-3;
        publish_scan(scan_pub_, data, data_intensity, ret, starts, scan_duration);
    }

    std::string port_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> stop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Global instance of the LidarNode
std::shared_ptr<LidarNode> node_instance;

// Signal handler
void signal_handler(int signum) {
    if (node_instance) {
        node_instance->stop_motor();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node_instance = std::make_shared<LidarNode>();

    // Register signal handler
    std::signal(SIGINT, signal_handler);

    rclcpp::spin(node_instance);
    rclcpp::shutdown();
    return 0;
}

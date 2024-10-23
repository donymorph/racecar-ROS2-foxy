#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <iomanip>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
}

using namespace std::chrono_literals;

static int data_length = 81;

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), io_service_(), serial_port_(io_service_) {
        this->declare_parameter<std::string>("port", "/dev/imu");
        this->declare_parameter<std::string>("model", "dony_imu_01");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");

        port_ = this->get_parameter("port").as_string();
        model_ = this->get_parameter("model").as_string();
        baud_ = this->get_parameter("baud").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        setup_serial(port_, baud_);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/magnetic_field", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10); //doesnt work, need to fix

        timer_ = this->create_wall_timer(
            50ms, std::bind(&ImuNode::read_and_publish, this));

        if (model_ == "dony_imu_01") {
            const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
            const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
            write(fd_, stop, 6);
            usleep(1000 * 1000);
            write(fd_, mode, 6);
            usleep(1000 * 1000);
            data_length = 40;
        }
    }

private:
    std::string port_, model_, frame_id_;
    int baud_;
    int fd_;

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // std::string data_to_string(const std::vector<uint8_t>& buf, size_t len) {
    //     std::ostringstream oss;
    //     for (size_t i = 0; i < len; ++i) {
    //         oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]) << " ";
    //     }
    //     return oss.str();
    // }
    void setup_serial(const std::string &port, int baud_rate) {
        serial_port_.open(port);
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        fd_ = open(port.c_str(), O_RDWR);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
        }
    }

    void read_and_publish() {
        if (!serial_port_.is_open()) return;

        std::vector<uint8_t> buf(256);
        boost::system::error_code ec;
        size_t len = serial_port_.read_some(boost::asio::buffer(buf), ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port: %s", ec.message().c_str());
            return;
        }
        // Print the raw data for debugging
        //std::string raw_data = data_to_string(buf, len);
        //#RCLCPP_INFO(this->get_logger(), "Raw IMU data: %s", raw_data.c_str());

        process_data(buf.data(), len);
    }

    void process_data(const uint8_t* data, size_t length) {
        for (size_t kk = 0; kk < length - 1; ++kk) {
            if (data[kk] == 0xA5 && data[kk + 1] == 0x5A) {
                uint8_t data_length = data[2];
                uint32_t checksum = 0;
                for (int i = 0; i < data_length - 1; ++i) {
                    checksum += (uint32_t)data[i + 2];
                }
                uint16_t check = checksum % 256;
                uint16_t check_true = data[data_length + 1];
            
                // if (check != check_true) {
                //     RCLCPP_WARN(this->get_logger(), "Checksum error, please wait. Calculated: %d, Received: %d", check, check_true);
                //     continue;
                // }

                //RCLCPP_INFO(this->get_logger(), "Checksum valid. Processing data...");
                            // Assuming data for Euler angles is available for X, Y, and Z axes
                float roll = -d2f_euler(data + 3);  // X-axis
                float pitch = -d2f_euler(data + 5); // Y-axis
                float yaw = -d2f_euler(data + 7);   // Z-axis
                Eigen::Vector3d ea0(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
                //Eigen::Vector3d ea0((d2f_euler(data + 3)) * M_PI / 180.0, 0, 0);
                Eigen::Matrix3d R;
                R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q;
                q = R;
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.orientation.w = q.w();
                imu_msg.orientation.x = q.x();
                imu_msg.orientation.y = q.y();
                imu_msg.orientation.z = q.z();
                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = frame_id_;
                imu_msg.angular_velocity.x = d2f_gyro(data + 15);
                imu_msg.angular_velocity.y = d2f_gyro(data + 17);
                imu_msg.angular_velocity.z = d2f_gyro(data + 19);
                imu_msg.linear_acceleration.x = d2f_acc(data + 11) * 9.81;
                imu_msg.linear_acceleration.y = d2f_acc(data + 9) * 9.81;
                imu_msg.linear_acceleration.z = d2f_acc(data + 13) * 9.81;

                imu_pub_->publish(imu_msg);

                sensor_msgs::msg::MagneticField mag_msg;
                mag_msg.magnetic_field.x = d2f_mag(data + 21);
                mag_msg.magnetic_field.y = d2f_mag(data + 23);
                mag_msg.magnetic_field.z = d2f_mag(data + 25);
                mag_msg.header.stamp = imu_msg.header.stamp;
                mag_msg.header.frame_id = imu_msg.header.frame_id;

                mag_pub_->publish(mag_msg);

                sensor_msgs::msg::NavSatFix gps_msg;
                gps_msg.header.stamp = imu_msg.header.stamp;
                gps_msg.header.frame_id = imu_msg.header.frame_id;
                gps_msg.latitude = d2f_latlon(data + 27);
                gps_msg.longitude = d2f_latlon(data + 31);
                gps_msg.altitude = d2f_latlon(data + 35);
                gps_pub_->publish(gps_msg);
            }
        }
    }

    static float d2f_acc(const uint8_t* a) {
        int16_t acc = a[0];
        acc = (acc << 8) | a[1];
        return static_cast<float>(acc) / 16384.0f;
    }

    static float d2f_gyro(const uint8_t* a) {
        int16_t acc = a[0];
        acc = (acc << 8) | a[1];
        return static_cast<float>(acc) / 32.8f;
    }

    static float d2f_mag(const uint8_t* a) {
        int16_t acc = a[0];
        acc = (acc << 8) | a[1];
        return static_cast<float>(acc) / 1.0f;
    }

    static float d2f_euler(const uint8_t* a) {
        int16_t acc = a[0];
        acc = (acc << 8) | a[1];
        return static_cast<float>(acc) / 10.0f;
    }

    static double d2f_latlon(const uint8_t* a) {
        int64_t high = a[0];
        high = (high << 8) | a[1];
        int64_t low = a[2];
        low = (low << 8) | a[3];
        return static_cast<double>((high << 8) | low) / 1e7;  // Assuming data is in 1e7 scaling
    }

    static double d2f_gpsvel(const uint8_t* a) {
        int16_t acc = a[0];
        acc = (acc << 8) | a[1];
        return static_cast<float>(acc) / 10.0f;
    }

    static float d2ieee754(const uint8_t* a) {
        union fnum {
            float f_val;
            uint8_t d_val[4];
        } f;
        memcpy(f.d_val, a, 4);
        return f.f_val;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

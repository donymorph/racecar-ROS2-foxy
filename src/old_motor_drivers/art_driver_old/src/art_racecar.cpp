// Created by Steven Zhang on 18-12-14.
// modified by adambaev doniyorbek on 24-10-21
// art racecar

#include "../include/art_racecar_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    double angle;
    angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
    send_cmd(uint16_t(twist->linear.x), uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400, data);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("art_driver");

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/car/cmd_vel", 1, TwistCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

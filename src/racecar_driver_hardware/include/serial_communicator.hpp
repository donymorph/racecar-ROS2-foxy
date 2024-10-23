#ifndef SERIAL_COMMUNICATOR_HPP
#define SERIAL_COMMUNICATOR_HPP

#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"

class SerialCommunicator
{
public:
  SerialCommunicator();
  ~SerialCommunicator();

  int openPort(const std::string & dev);
  int configurePort(int baudrate, int bits, char parity, int stop);
  int sendCmd(uint16_t motor_pwm, uint16_t servo_pwm);

private:
  int fd_;
  int check_uint(uint16_t param);
  void print_send_buff(unsigned char * buffer);
};

#endif  // SERIAL_COMMUNICATOR_HPP
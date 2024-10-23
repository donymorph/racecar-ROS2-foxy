#include "../include/serial_communicator.hpp"
#include <cstring>
#include <cerrno>

SerialCommunicator::SerialCommunicator() : fd_(-1) {}

SerialCommunicator::~SerialCommunicator()
{
  if (fd_ != -1)
  {
    close(fd_);
  }
}

int SerialCommunicator::openPort(const std::string & dev)
{
  fd_ = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Cannot open the serial port!");
    return -1;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("SerialCommunicator"), "Opened serial port successfully!");
    return 1;
  }
}

int SerialCommunicator::configurePort(int baudrate, int bits, char parity, int stop)
{
  if (fcntl(fd_, F_SETFL, 0) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "fcntl failed");
    return -1;
  }

  struct termios newtio, oldtio;
  if (tcgetattr(fd_, &oldtio) != 0)
  {
    perror("SetupSerial 1");
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Failed to get serial port attributes");
    return -1;
  }
  bzero(&newtio, sizeof(newtio));

  // Set baud rate
  cfsetispeed(&newtio, baudrate);
  cfsetospeed(&newtio, baudrate);

  // Set data bits
  newtio.c_cflag &= ~CSIZE;
  if (bits == 7)
    newtio.c_cflag |= CS7;
  else
    newtio.c_cflag |= CS8;

  // Set parity
  switch (parity)
  {
    case 'N':
    case 'n':
      newtio.c_cflag &= ~PARENB;
      newtio.c_iflag &= ~INPCK;
      break;
    case 'E':
    case 'e':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      newtio.c_iflag |= INPCK;
      break;
    case 'O':
    case 'o':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= INPCK;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Unsupported parity");
      return -1;
  }

  // Set stop bits
  if (stop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (stop == 2)
    newtio.c_cflag |= CSTOPB;
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Unsupported stop bits");
    return -1;
  }

  // Set other options
  newtio.c_cflag |= (CLOCAL | CREAD);
  newtio.c_lflag = 0;
  newtio.c_oflag = 0;
  newtio.c_iflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &newtio) != 0)
  {
    perror("Failed to set serial port attributes");
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Failed to set serial port attributes");
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialCommunicator"), "Serial port configured successfully");

  return 0;
}

int SerialCommunicator::sendCmd(uint16_t motor_pwm, uint16_t servo_pwm)
{
  unsigned char send_buffer[8];
  send_buffer[0] = 0x00;
  send_buffer[1] = 0xAA;
  send_buffer[2] = motor_pwm & 0xFF;
  send_buffer[3] = motor_pwm >> 8;
  send_buffer[4] = servo_pwm & 0xFF;
  send_buffer[5] = servo_pwm >> 8;
  send_buffer[6] = check_uint(motor_pwm) + check_uint(servo_pwm);
  send_buffer[7] = 0x55;

  // Use ::write to call the global write() function
  ssize_t bytes_written = ::write(fd_, send_buffer, 8);
  if (bytes_written == -1)
  {
    perror("Write to serial port failed");
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunicator"), "Write to serial port failed: %s", strerror(errno));
    return -1;
  }
  else
  {
    RCLCPP_DEBUG(rclcpp::get_logger("SerialCommunicator"), "Sent %ld bytes to serial port", bytes_written);
    // Uncomment the following line to debug the send buffer
    //print_send_buff(send_buffer);
  }
  return send_buffer[6];
}

int SerialCommunicator::check_uint(uint16_t param)
{
  uint8_t low = param & 0xFF;
  uint8_t high = (param >> 8) & 0xFF;
  return low + high;
}

void SerialCommunicator::print_send_buff(unsigned char * buffer)
{
  //printf("Start Byte: %02x\n", buffer[0]);
  //printf("Header: %02x\n", buffer[1]);
  printf("Motor PWM: %02x %02x (Decimal: %d)\n", buffer[2], buffer[3], (buffer[3] << 8) | buffer[2]);
  printf("Servo PWM: %02x %02x (Decimal: %d)\n", buffer[4], buffer[5], (buffer[5] << 8) | buffer[4]);
  //printf("Checksum: %02x\n", buffer[6]);
  //printf("End Byte: %02x\n", buffer[7]);
  puts("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

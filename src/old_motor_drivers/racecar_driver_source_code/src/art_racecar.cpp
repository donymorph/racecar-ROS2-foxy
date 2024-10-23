#include "../include/art_racecar_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

// Global file descriptor for the serial port
int fd;

// Function to open the serial device
int Open_Serial_Dev(char *dev) {
    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        puts("Cannot open the Serial port!");
        return -1;
    } else {
        puts("Opened Serial port successfully!");
        return 1;
    }
}

// Function to set the serial port options
int set_opt(int fd, int speed, int bits, char parity, int stop) {
    struct termios newtio, oldtio;
    int i, baudrate_constant;

    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));

    // Map the baud rate
    int speed_arr[] = {B921600, B460800, B230400, B115200, B57600, B38400,
                       B19200,  B9600,   B4800,   B2400,   B1200,  B300};
    int name_arr[] = {921600, 460800, 230400, 115200, 57600, 38400,
                      19200,  9600,   4800,   2400,   1200,  300};
    for (i = 0; i < sizeof(name_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
            baudrate_constant = speed_arr[i];
            break;
        }
    }
    if (i == sizeof(name_arr) / sizeof(int)) {
        fprintf(stderr, "Unsupported baud rate: %d\n", speed);
        return -1;
    }

    // Set baud rate
    cfsetispeed(&newtio, baudrate_constant);
    cfsetospeed(&newtio, baudrate_constant);

    // Enable receiver and set local mode
    newtio.c_cflag |= (CLOCAL | CREAD);

    // Set data bits
    newtio.c_cflag &= ~CSIZE;
    if (bits == 7)
        newtio.c_cflag |= CS7;
    else
        newtio.c_cflag |= CS8;

    // Set parity
    switch (parity) {
        case 'N':
        case 'n':
            newtio.c_cflag &= ~PARENB;   // Clear parity enable
            newtio.c_iflag &= ~INPCK;    // Disable input parity checking
            break;
        case 'E':
        case 'e':
            newtio.c_cflag |= PARENB;    // Enable parity
            newtio.c_cflag &= ~PARODD;   // Even parity
            newtio.c_iflag |= INPCK;     // Enable input parity checking
            break;
        case 'O':
        case 'o':
            newtio.c_cflag |= PARENB;    // Enable parity
            newtio.c_cflag |= PARODD;    // Odd parity
            newtio.c_iflag |= INPCK;     // Enable input parity checking
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return -1;
    }

    // Set stop bits
    if (stop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (stop == 2)
        newtio.c_cflag |= CSTOPB;
    else {
        fprintf(stderr, "Unsupported stop bits\n");
        return -1;
    }

    // Set input flags
    newtio.c_iflag = 0;

    // Set output flags
    newtio.c_oflag = 0;

    // Set local flags
    newtio.c_lflag = 0; // Non-canonical mode, no echo

    // Set control characters
    newtio.c_cc[VTIME] = 0; // No inter-character timer
    newtio.c_cc[VMIN] = 0;  // Non-blocking read

    // Flush the input and output buffers
    tcflush(fd, TCIFLUSH);

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        perror("Failed to set serial port attributes");
        return -1;
    }

    printf("Serial port set to baud rate %d\n", speed);
    return 0;
}



// Function to initialize the racecar
int art_racecar_init(int baudrate, char *port) {
    if (Open_Serial_Dev(port) == -1) {
        return -1;
    }
    
    if (fcntl(fd, F_SETFL, 0) < 0) {
        puts("fcntl failed");
        return -1;
    }

    if (set_opt(fd, baudrate, 8, 'N', 1) == -1) {
        puts("failed");
        return -1;
    }

    return 1;
}

// Function to check unsigned integers (used internally)
int check_uint(uint16_t param) {
    uint8_t low = param & 0xFF;
    uint8_t high = (param >> 8) & 0xFF;
    return low + high;
}

// Function to send a command to the racecar
unsigned char send_cmd(uint16_t motor_pwm, uint16_t servo_pwm) {
    unsigned char send_buffer[8];
    send_buffer[0] = 0x00;
    send_buffer[1] = 0xAA;
    send_buffer[2] = motor_pwm & 0xFF;
    send_buffer[3] = motor_pwm >> 8;
    send_buffer[4] = servo_pwm & 0xFF;
    send_buffer[5] = servo_pwm >> 8;
    send_buffer[6] = check_uint(motor_pwm) + check_uint(servo_pwm);
    send_buffer[7] = 0x55;

    ssize_t bytes_written = write(fd, send_buffer, 8);
    if (bytes_written == -1) {
        perror("Write to serial port failed");
    } else {
        printf("Sent %ld bytes to serial port\n", bytes_written);
        print_send_buff(send_buffer);
    }
    return send_buffer[6];
}


// Function to print the send buffer (for debugging)
void print_send_buff(unsigned char *buffer) {
    printf("Start Byte: %02x\n", buffer[0]);
    printf("Header: %02x\n", buffer[1]);
    printf("Motor PWM: %02x %02x (Decimal: %d)\n", buffer[2], buffer[3], (buffer[3] << 8) | buffer[2]);
    printf("Servo PWM: %02x %02x (Decimal: %d)\n", buffer[4], buffer[5], (buffer[5] << 8) | buffer[4]);
    printf("Checksum: %02x\n", buffer[6]);
    printf("End Byte: %02x\n", buffer[7]);
    puts("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}


// Callback function for processing twist messages
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist) {
    double angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
    printf("Received Twist message: linear.x = %f, angular.z = %f\n", twist->linear.x, twist->angular.z);
    printf("Calculated motor_pwm = %u, servo_pwm = %u\n", uint16_t(twist->linear.x), uint16_t(angle));
    send_cmd(uint16_t(twist->linear.x), uint16_t(angle));
}


// Main function
int main(int argc, char** argv) {
    char data[] = "/dev/car";
    if (art_racecar_init(38400, data) == -1) {
        fprintf(stderr, "Failed to initialize the racecar.\n");
        return -1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("racecar_driver_node");

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/car/cmd_vel", rclcpp::QoS(10), TwistCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

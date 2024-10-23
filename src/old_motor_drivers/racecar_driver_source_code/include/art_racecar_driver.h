#ifndef ART_RACECAR_DRIVER
#define ART_RACECAR_DRIVER
#include <stdint.h>
#include <unistd.h>
#include <termios.h>

#if defined(__cplusplus)
extern "C" {
#endif

// Function declarations
int Open_Serial_Dev(char *dev);  // Opens the serial device

int set_opt(int fd, int speed, int bits, char parity, int stop);  // Configures the serial port

int art_racecar_init(int speed, char *dev);  // Initializes the racecar with the given baudrate and device

unsigned char send_cmd(uint16_t motor_pwm, uint16_t servo_pwm);  // Sends a command (motor PWM, servo PWM)

void print_send_buff(unsigned char *buffer);  // Prints the send buffer for debugging

int check_uint(uint16_t param);  // Checks and returns the sum of the low and high bytes of a uint16_t

#if defined(__cplusplus)
}
#endif
#endif

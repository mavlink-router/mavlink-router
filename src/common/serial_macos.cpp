#include "serial.h"
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#include <termios.h>

int init_serial(int fd, const char *path)
{
    struct termios tty = {};

    int ret = tcgetattr(fd, &tty);
    if (ret < 0) {
        return ret;
    }

    // Input flags
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_iflag &= ~(IXON | IXOFF); // No software flow control
    tty.c_iflag &= ~INPCK;          // No parity checking
    tty.c_iflag |= IGNPAR;          // Ignore parity errors

    // Output flags
    tty.c_oflag &= ~OPOST; // No post processing

    // Control flags
    tty.c_cflag |= (CLOCAL | CREAD);   // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~(PARENB | PARODD); // No parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;           // No hardware flow control
    tty.c_cflag &= ~CSIZE;             // Clear data size bits
    tty.c_cflag |= CS8;                // 8 data bits

    // Local flags
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Control characters
    tty.c_cc[VMIN] = 0;  // Non-blocking read
    tty.c_cc[VTIME] = 0; // No timeout

    ret = tcsetattr(fd, TCSANOW, &tty);
    if (ret < 0) {
        return ret;
    }

    tcflush(fd, TCIOFLUSH);

    return 0;
}

int set_serial_speed(int fd, uint32_t baudrate)
{
    speed_t speed = static_cast<speed_t>(baudrate);

    int ret = ioctl(fd, IOSSIOSPEED, &speed);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int set_serial_flow_control(int fd, bool enabled)
{
    struct termios tty = {};

    int ret = tcgetattr(fd, &tty);
    if (ret < 0) {
        return ret;
    }

    if (enabled) {
        tty.c_cflag |= CRTSCTS;
    } else {
        tty.c_cflag &= ~CRTSCTS;
    }

    ret = tcsetattr(fd, TCSANOW, &tty);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

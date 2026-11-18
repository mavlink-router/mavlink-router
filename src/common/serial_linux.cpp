#include "log.h"
#include "serial.h"
#include "xtermios.h"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

int init_serial(int fd, const char *path)
{
    int ret = 0;
    ret = reset_uart(fd);
    if (ret < 0) {
        log_error("Could not reset uart on %s", path);
        return ret;
    }

    struct termios2 tc = {};

    ret = ioctl(fd, TCGETS2, &tc);
    if (ret < 0) {
        log_error("Could not get termios2 on %s (%m)", path);
        return ret;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    tc.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

    /* never send SIGTTOU*/
    tc.c_lflag &= ~(TOSTOP);

    /* disable flow control */
    tc.c_cflag &= ~(CRTSCTS);
    tc.c_cflag &= ~(CSIZE | PARENB);

    /* ignore modem control lines */
    tc.c_cflag |= CLOCAL;

    /* 8 bits */
    tc.c_cflag |= CS8;

    /* we use epoll to get notification of available bytes */
    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;

    ret = ioctl(fd, TCSETS2, &tc);
    if (ret < 0) {
        log_error("Could not set terminal attributes on %s (%m)", path);
        return ret;
    }

    // For Linux, set high speed polling at the chip
    // level. Since this routine relies on a USB latency
    // change at the chip level it may fail on certain
    // chip sets if their driver does not support this
    // configuration request

    {
        struct serial_struct serial_ctl;

        ret = ioctl(fd, TIOCGSERIAL, &serial_ctl);
        if (ret < 0) {
            log_warning("Error while trying to read serial port configuration on %s: %m", path);
        } else {
            serial_ctl.flags |= ASYNC_LOW_LATENCY;

            ret = ioctl(fd, TIOCSSERIAL, &serial_ctl);
            if (ret < 0) {
                if (errno != ENODEV && errno != ENOTTY) {
                    log_warning("Error while trying to write serial port latency on %s: %m", path);
                }
            }
        }
    }

    ret = ioctl(fd, TCFLSH, TCIOFLUSH);
    if (ret < 0) {
        log_error("Could not flush terminal on %s (%m)", path);
        return ret;
    }

    return 0;
}

int set_serial_speed(int fd, uint32_t baudrate)
{
    if (fd < 0) {
        return -1;
    }

    struct termios2 tc = {};

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("Could not get termios2 (%m)");
        return -1;
    }

    /* speed is configured by c_[io]speed */
    tc.c_cflag &= ~CBAUD;
    tc.c_cflag |= BOTHER;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes (%m)");
        return -1;
    }

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("Could not flush terminal (%m)");
        return -1;
    }

    return 0;
}

int set_serial_flow_control(int fd, bool enabled)
{
    if (fd < 0) {
        return -1;
    }

    struct termios2 tc = {};

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("Could not get termios2 (%m)");
        return -1;
    }

    if (enabled) {
        tc.c_cflag |= CRTSCTS;
    } else {
        tc.c_cflag &= ~CRTSCTS;
    }

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes (%m)");
        return -1;
    }

    return 0;
}

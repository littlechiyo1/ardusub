
#ifndef SERIAL_H
#define SERIAL_H

#include <errno.h>
#include <fcntl.h>
#include <linux/rtc.h>
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

void serial_close(int fd);

int serial_open(const char *dev, unsigned int baud);

int serial_read_data(int fd, char *val, int len);

int serial_write_data(int fd, const char *val, int len);

#endif

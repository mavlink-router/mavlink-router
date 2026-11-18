#pragma once

#include <time.h>

#ifdef __APPLE__

struct itimerspec {
    struct timespec it_interval;
    struct timespec it_value;
};

#endif

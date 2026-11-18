#pragma once

#include <sys/socket.h>

#ifdef __APPLE__
#    ifndef IPV6_ADD_MEMBERSHIP
#        define IPV6_ADD_MEMBERSHIP IPV6_JOIN_GROUP
#    endif
#endif

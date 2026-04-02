#pragma once

#ifndef __APPLE__
#    include <endian.h>
#else
#    include <libkern/OSByteOrder.h>
#    ifndef htobe64
#        define htobe64(x) OSSwapHostToBigInt64(x)
#    endif
#endif

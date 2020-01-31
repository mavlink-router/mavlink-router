#include "mainloop.h"

#include <gtest/gtest.h>

TEST(MainLoopTest, termination) {
    Mainloop mainloop;

    mainloop.request_exit();

    mainloop.loop();
}

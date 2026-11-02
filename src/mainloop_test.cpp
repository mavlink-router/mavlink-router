#include "mainloop.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include <gtest/gtest.h>
#pragma GCC diagnostic pop

TEST(MainLoopTest, termination)
{
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(0);
    int ret = mainloop.loop();
    EXPECT_EQ(0, ret);
    mainloop.teardown();
}

TEST(MainLoopTest, wrong_termination)
{
    Mainloop &mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(-1);
    int ret = mainloop.loop();
    EXPECT_EQ(-1, ret);
    mainloop.teardown();
}

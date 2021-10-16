#include "mainloop.h"

#include <gtest/gtest.h>

TEST(MainLoopTest, termination) {
    Mainloop& mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(0);
    int ret = mainloop.loop();
    EXPECT_EQ(0, ret);
}

TEST(MainLoopTest, wrong_termination) {
    Mainloop& mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit(-1);
    int ret = mainloop.loop();
    EXPECT_EQ(-1, ret);
}

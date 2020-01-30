#include "mainloop.h"

#include <gtest/gtest.h>

TEST(MainLoopTest, termination) {
    Mainloop& mainloop = Mainloop::init();
    mainloop.open();
    mainloop.request_exit();
    mainloop.loop();
}

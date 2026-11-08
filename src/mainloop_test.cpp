#include "mainloop.h"

#include <stdlib.h>
#include <unistd.h>

#include <gtest/gtest.h>

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

TEST(MainLoopTest, log_endpoint_keeps_unknown_target_accounting)
{
    char logs_dir[] = "/tmp/mavlink-router-test-XXXXXX";
    ASSERT_NE(mkdtemp(logs_dir), nullptr);

    Mainloop &mainloop = Mainloop::init();
    mainloop.open();

    // Only the passive log endpoints, so nothing knows any routable target
    Configuration config{};
    config.tcp_port = 0;
    config.log_config.logs_dir = logs_dir;
    config.log_config.mavlink_dialect = LogOptions::MavDialect::Common;
    config.log_config.log_telemetry = true;
    ASSERT_TRUE(mainloop.add_endpoints(config));

    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {};
    buffer msg{};
    msg.data = data;
    msg.len = sizeof(data);
    msg.curr.msg_id = MAVLINK_MSG_ID_COMMAND_LONG;
    msg.curr.src_sysid = 1;
    msg.curr.src_compid = 1;

    // point-to-point message to a system no endpoint knows: still unknown
    msg.curr.target_sysid = 77;
    msg.curr.target_compid = 1;
    uint32_t unknown_before = mainloop.msg_to_unknown_count();
    mainloop.route_msg(&msg);
    EXPECT_EQ(mainloop.msg_to_unknown_count(), unknown_before + 1);

    // broadcast message: not counted as unknown (unchanged behavior)
    msg.curr.target_sysid = 0;
    msg.curr.target_compid = 0;
    unknown_before = mainloop.msg_to_unknown_count();
    mainloop.route_msg(&msg);
    EXPECT_EQ(mainloop.msg_to_unknown_count(), unknown_before);

    mainloop.clear_endpoints();
    mainloop.teardown();
    rmdir(logs_dir);
}

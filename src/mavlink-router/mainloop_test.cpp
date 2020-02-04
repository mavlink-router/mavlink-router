#include "mainloop.h"

#include <cstring>

#include <gtest/gtest.h>

class MainLoopTest : public ::testing::Test {
public:
    static struct endpoint_config make_udp_endpoint_config(int port, bool coalesce)
    {
        struct endpoint_config cfg {};
        // XXX: need non-const strings here, despite the fact that this is
        // strongly discouraged in general: config data structure is defined
        // this way
        static char name[] = "dummy";
        static char address[] = "127.0.0.1";
        cfg.type = Udp;
        cfg.name = name;
        // XXX: hard-coding address & port in test is ill-advised; API does not
        // allow injection of sockets.
        cfg.address = address;
        cfg.port = port;
        cfg.retry_timeout = 0;
        // XXX: not clear about the name of this variable -- it causes socket
        // to be bound to specified port
        cfg.eavesdropping = true;
        if (coalesce) {
            cfg.coalesce_bytes = 100;
            cfg.coalesce_ms = 1;
        }
        cfg.filter = nullptr;
        static char nodelay[] = "75,76,77";
        cfg.coalesce_nodelay = nodelay;

        return cfg;
    }

    static struct options make_single_endpoint_options(endpoint_config* cfg)
    {
        struct options opts{};
        opts.endpoints = cfg;
        return opts;
    }

    static std::pair<int, sockaddr_in> make_scratch_udp_socket()
    {
        int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = 0;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        ::bind(fd, reinterpret_cast<const struct sockaddr*>(&addr), sizeof(addr));
        socklen_t addrlen = sizeof(addr);
        ::getsockname(fd, reinterpret_cast<struct sockaddr*>(&addr), &addrlen);

        return {fd, addr};
    }
};

TEST_F(MainLoopTest, termination)
{
    Mainloop mainloop;

    mainloop.request_exit();

    mainloop.loop();
}

TEST_F(MainLoopTest, create_udp_endpoint)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, false);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
}

TEST_F(MainLoopTest, direct_udp_endpoint_send)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, false);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;

    // Set up and grab one udp endpoint
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
    UdpEndpoint* udp_endpoint = dynamic_cast<UdpEndpoint *>(mainloop.endpoints()[0].get());
    ASSERT_NE(nullptr, udp_endpoint);

    int sock;
    std::tie(sock, udp_endpoint->sockaddr) = make_scratch_udp_socket();

    char data[17] = "0123456789abcdef";
    struct buffer buf = {16, reinterpret_cast<uint8_t*>(data)};
    udp_endpoint->write_msg(&buf);

    char recvbuf[1024];
    ssize_t count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(16, count);
    EXPECT_EQ(0, std::memcmp("0123456789abcdef", recvbuf, 16));

    ::close(sock);
}


TEST_F(MainLoopTest, direct_udp_endpoint_send_coalesce_time_trigger)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, true);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;

    // Set up and grab one udp endpoint
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
    UdpEndpoint* udp_endpoint = dynamic_cast<UdpEndpoint *>(mainloop.endpoints()[0].get());
    ASSERT_NE(nullptr, udp_endpoint);

    int sock;
    std::tie(sock, udp_endpoint->sockaddr) = make_scratch_udp_socket();

    char data[17] = "0123456789abcdef";
    struct buffer buf = {16, reinterpret_cast<uint8_t*>(data)};
    udp_endpoint->write_msg(&buf);

    char recvbuf[1024];
    ssize_t count = ::recv(sock, recvbuf, 1024, MSG_DONTWAIT);

    EXPECT_EQ(-1, count) << "UDP packet coalescing shouldn't have allowed a send yet";
    EXPECT_EQ(EWOULDBLOCK, errno);

    // allow UDP packet coalescing to expire
    // XXX: make the timer expire explicitly, instead of waiting for timeout
    mainloop.run_single(100);

    count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(16, count);
    EXPECT_EQ(0, std::memcmp("0123456789abcdef", recvbuf, 16));

    ::close(sock);
}

TEST_F(MainLoopTest, direct_udp_endpoint_send_coalesce_size_trigger)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, true);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;

    // Set up and grab one udp endpoint
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
    UdpEndpoint* udp_endpoint = dynamic_cast<UdpEndpoint *>(mainloop.endpoints()[0].get());
    ASSERT_NE(nullptr, udp_endpoint);

    int sock;
    std::tie(sock, udp_endpoint->sockaddr) = make_scratch_udp_socket();

    char data[110] = {};
    for (int i = 1; i < 110; i++) data[i] = i;

    const struct buffer buf = {110, reinterpret_cast<uint8_t*>(data)};
    udp_endpoint->write_msg(&buf);

    char recvbuf[1024];
    ssize_t count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(110, count);
    EXPECT_EQ(0, std::memcmp(data, recvbuf, 110));

    ::close(sock);
}

TEST_F(MainLoopTest, direct_udp_endpoint_send_coalesce_size_trigger_from_second)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, true);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;

    // Set up and grab one udp endpoint
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
    UdpEndpoint* udp_endpoint = dynamic_cast<UdpEndpoint *>(mainloop.endpoints()[0].get());
    ASSERT_NE(nullptr, udp_endpoint);

    int sock;
    std::tie(sock, udp_endpoint->sockaddr) = make_scratch_udp_socket();

    char data[55] = {};
    for (int i = 1; i < 55; i++) data[i] = i;

    struct buffer buf = {55, reinterpret_cast<uint8_t*>(data)};
    udp_endpoint->write_msg(&buf);

    char recvbuf[1024];
    ssize_t count = ::recv(sock, recvbuf, 1024, MSG_DONTWAIT);

    EXPECT_EQ(-1, count) << "UDP packet coalescing shouldn't have allowed a send yet";
    EXPECT_EQ(EWOULDBLOCK, errno);

    buf.len = 50;
    udp_endpoint->write_msg(&buf);

    count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(55, count);
    EXPECT_EQ(0, std::memcmp(data, recvbuf, 55));

    // allow UDP packet coalescing to expire
    // XXX: make the timer expire explicitly, instead of waiting for timeout
    mainloop.run_single(100);

    memset(recvbuf, 0, 1024);
    count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(50, count);
    EXPECT_EQ(0, std::memcmp(data, recvbuf, 50));

    ::close(sock);
}

TEST_F(MainLoopTest, direct_udp_endpoint_send_coalesce_nodelay)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777, true);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;

    // Set up and grab one udp endpoint
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
    UdpEndpoint* udp_endpoint = dynamic_cast<UdpEndpoint *>(mainloop.endpoints()[0].get());
    ASSERT_NE(nullptr, udp_endpoint);

    int sock;
    std::tie(sock, udp_endpoint->sockaddr) = make_scratch_udp_socket();

    char data[17] = "0123456789abcdef";
    struct buffer buf = {16, reinterpret_cast<uint8_t*>(data)};
    udp_endpoint->write_msg(&buf);
    udp_endpoint->postprocess_msg(0,0,0,0,76);

    char recvbuf[1024];
    ssize_t count = ::recv(sock, recvbuf, 1024, 0);

    EXPECT_EQ(16, count);
    EXPECT_EQ(0, std::memcmp("0123456789abcdef", recvbuf, 16));

    ::close(sock);
}

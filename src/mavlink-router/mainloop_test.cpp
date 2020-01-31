#include "mainloop.h"

#include <cstring>

#include <gtest/gtest.h>

class MainLoopTest : public ::testing::Test {
public:
    static struct endpoint_config make_udp_endpoint_config(int port)
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
        cfg.filter = nullptr;
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
    struct endpoint_config cfg = make_udp_endpoint_config(7777);
    struct options opts = make_single_endpoint_options(&cfg);

    Mainloop mainloop;
    mainloop.add_endpoints(mainloop, &opts);
    ASSERT_EQ(1, mainloop.endpoints().size());
}

TEST_F(MainLoopTest, direct_udp_endpoint_send)
{
    struct endpoint_config cfg = make_udp_endpoint_config(7777);
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

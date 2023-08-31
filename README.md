# MAVLink Router

MAVLink Router is an application to distribute [MAVLink](https://mavlink.io/en/)
messages between multiple endpoints (connections). It distributes packets to a
single port or multiple endpoints depending on the target address. Connections
can be made via UART, UDP or TCP (see the [endpoints chapter](#endpoints) for
details).


## Compilation and Installation

In order to compile you need the following packages:

  - GCC or Clang compiler
  - meson >= 0.57 and ninja-build
  - C and C++ standard libraries

### Fetch dependencies

We currently depend on the mavlink C library. The corresponding submodule
should be fetched:

    $ git submodule update --init --recursive

We need some additional packages as build dependencies. Packages for some
distros:

Debian/Ubuntu:

    $ sudo apt install git meson ninja-build pkg-config gcc g++ systemd

Fedora:

    $ sudo dnf install git meson gcc g++ systemd

Archlinux:

    $ sudo pacman -S git meson gcc systemd

Note that meson package must be version 0.55 or later. If your package manager
does not have this version, a more recent version can be downloaded via pip:

    $ sudo pip3 install meson

If using this method, meson must _not_ be installed with ``--user``.

### Build

The build system follows the usual configure/build/install cycle.
Configuration is needed to be done only once.

A typical configuration for a x86-64 system is shown below:

    $ meson setup build .

By default systemd integration is enabled. In a system without systemd you
should pass the service directory with `-Dsystemdsystemunitdir=/usr/lib/systemd/system`.
By default the build type is settled to "debugoptimized". It can be changed
with `--buildtype=release`, see [Meson documentation](https://mesonbuild.com/Builtin-options.html#core-options)
for more options.

Installation location can be changed using the -Dprefix option to `meson setup`.

Build:

    $ ninja -C build

Install:

    $ sudo ninja -C build install

Or in order to install to another root directory:

    $ sudo DESTDIR=/tmp/root/dir ninja -C build install


## Running

There are two ways to configure mavlink-router: Configuration file(s) and
command line parameters. You can use just config files or just CLI options or
even both at the same time. The CLI options will be merged with the settings
from the config file in the latter case.  
The configuration file gives more fine-grained control over the endpoints while
the CLI options enable a quick configuration. When using the systemd unit
added by the install step, it's recommended to use the configuration file
instead of editing the systemd file.

### Conf File

By default, mavlink-routerd looks for the `/etc/mavlink-router/main.conf` file.
The file location can be overriden via a `MAVLINK_ROUTERD_CONF_FILE` environment
variable, or via the `-c` CLI switch when running mavlink-routerd. A
description of the config file syntax and all parameters can be found in the
[examples/config.sample](examples/config.sample) file.

### Conf Directory

Besides the default conf file, it's also possible to use a directory in where
to put some extra configuration files. Files in this directory will be read in
alphabetical order, and can add or override configurations found in previous
files.

By default, `/etc/mavlink-router/config.d` is the directory, but it can be
overriden via a `MAVLINK_ROUTERD_CONF_DIR` environment variable, or via the `-d`
switch when running mavlink-routerd.

### CLI Parameters


Please see the output of `mavlink-routerd --help` for the full list of command
line options. The most important facts are:

  - The TCP server is enabled by default
  - TCP and UDP endpoints can be added multiple times
  - UDP endpoints added with the `-e` option are started in `normal` mode
    (sending data to the specified address and port)
  - The last parameter (without a key) can either be one UART device or an UDP
    connection. This UDP endpoint will be started in `server` mode (waiting for
    an incoming connection)!


To route mavlink packets from UART `ttyS1` to 2 other UDP endpoints, use the
following command:

    $ mavlink-routerd -e 192.168.7.1:14550 -e 127.0.0.1:14550 /dev/ttyS1:1500000

The `1500000` after the colon in `/dev/ttyS1:1500000` sets the UART baudrate.
See more options with `mavlink-routerd --help`.

It's also possible to route mavlinks packets from an incoming UDP connection
instead of UART:

    $ mavlink-routerd -e 192.168.7.1:14550 -e 127.0.0.1:14550  0.0.0.0:24550

Additionally, mavlink-router also listens on port 5760 for TCP connections by
default. Any client connecting to that port will be able to send and receive
MAVLink data.

IPv6 addresses must be enclosed in square brackets like this: `[::1]`. The port
number can be specified in the same way, as with IPv4 then: `[::1]:14550`. Both
unicast and multicast addresses should be handled properly and the interface
for a link-local address is auto-detected.


## Detailed Description of Capabilities

To understand how MAVLink Router forwards messages between endpoints, it
important to know, that a MAVLink message has to have a sender address, but
only some messages have a target address (e.g. parameter requests). The sender
and target address consist of a system and component ID each, so one UAV can
have a flight controller as well as other services running on a companion
computer with individual component IDs, but sharing the same system ID. In the
target address, a component ID of 0 is used to broadcast to all components on a
system, a sysID of 0 broadcasts to all systems.

### Endpoints
MAVLink Router supports three basic types of endpoints: UART, UDP link and TCP
client. Additionally, it'll act as a TCP server for dynamic clients (if not
explicitly deactivated).

Endpoint types (see [examples/config.sample](examples/config.sample) for the
config file format):

  - UART: For telemetry radios or other serial links
    * Configuration: UART device path/name and baudrate
    * Behavior: Data is received and sent without waiting for incoming data first
  - UDP:
    * Configuration: Mode (client or server), IP address and port
    * Behavior in client mode: Endpoint is configured with a target IP and port
      combination. So MAVLink messages can be sent directly after startup, but
      will only be recevied after the first message was received by the remote
      side, it doesn't know our IP and port otherwise.  
      When using any non-unicast IP address, e.g. an IPv4 broadcast or IPv6
      local network multicast (ff02::1), messages will be "broadcasted" until
      somebody sends data back. From then on, UDP packets will only be sent to
      the specific unicast IP of the answering device. When no MAVlink messages
      were received for 5 seconds, the endpoint switches back to "broadcast"
      mode (using the configured non-unicast IP address).
    * Behavior in server mode: Endpoint is configured with a listening port and
      IP address. This is essentially the opposite of client mode. Messages can
      be received directly after startup, but we can only send messages out
      after the first received message, because we don't know the remote IP and
      port otherwise.  
      MAVLink messages are always sent to the IP and port from which the last
      incoming message was received.
  - TCP Client:
    * Configuration: Target IP address and port, reconnection interval in case
      of disconnection
    * Behavior: Data is received and sent right after the TCP session is
      established

Defining endpoints:

  - Endpoints are created by one of these methods:
    * An endpoint is defined in the configuration file
    * An endpoint is defined by the corresponding command line option
    * A TCP client has connected to the TCP server port
  - Endpoint are destoyed, when
    * A TCP client disconnects from the TCP server port
    * MAVLink Router is terminated
    * (This means that UART, UDP and TCP client endpoints are never destroyed
      during runtime.)

### Message Routing
In general, each message received on one endpoint is delivered to all endpoints
in which that target system/component has been seen. If it's a broadcast
message, it's delivered to all endpoints. A message is never sent back to the
same endpoint it came from.  
Details on broadcast rules can be found in the official
[MAVLink documentation](http://mavlink.io/en/guide/routing.html).

Routing rules:

  - Each endpoint remembers from which systems (system and component ID) it has
    received messages during it's whole lifetime. (See [endpoints chapter](#endpoints)
    for information when an endpoint is created and destroyed.)
  - A message received on one endpoint is offered to all endpoints but the one
    it was received on. An endpoint will:
    1. Reject the message, if message's sender address **is** in the list of
      connected systems on this endpoint (to prevent message loops)
    2. Reject the message based on the outgoing message filters (if enabled)
    3. Accept the message, if it's targeted to any of the systems in the list
      of connected systems on this endpoint. Broadcast rules apply when
      checking if the targeted is reachable via this endpoint. Messages without
      target address count as broadcast.  
      If the list of connected systems is empty, only system-ID broadcast
      messages will be sent, but no component-ID broadcasts since the targeted
      system isn't known to be reachable via this endpoint.
    4. Reject all other messages

Message filters:

  - There are two points where messages can be filtered on each endpoint:
    - **In**: Messages which are received (from the outside) on this endpoint are dropped or allowed based on the respecitive filter rules before they'll be routed to other endpoints
    - **Out**: Messages are dropped or allowed based on the endpoint's filter rules before being transmitted. So this is after internal routing (see "routing rules" chapter above).
  - A message filter can be based on one of these message identifiers:
    - **MsgId**: Filter message based on it's MAVLink message ID (message type like HEARTBEAT)
    - **SrcSys**: Filter message based on it's MAVLink source system ID
    - **SrcComp**: Filter message based on it's MAVLink source component ID
  - And a message filter can either be a block- or allow-list:
    - **Block**: Discard all messages matching the respective identifier (and allow all other ones)
    - **Allow**: Allow all messages matching the respective identifier (and discard all other ones)
    - Note that while using "Allow" and "Block" filters on the same identifier 
    within an endpoint doesn't make sense, using them on different identifiers 
    can be useful (for example, allowing only specific outgoing SysID, and
    blocking this system from sending some unwanted message IDs).
  - So a filter might be named `AllowMsgIdOut` to only allow messages with the listed message ID to be transmitted on that endpoint. See the example config [examples/config.sample](examples/config.sample) for the exact name of each filter parameter.

Message de-duplication:

  - If enabled, each incoming message is checked, whether another copy was
    already received the last `DeduplicationPeriod` milliseconds ago. If it's
    already known, the message will be dropped as it was never received and the
    timeout counter for that message will be reset. Messages are identified via
    their `std::hash` value of the full MAVLink message including it's header.  
    As long as no message with exactly the same header sequence number and
    content is received during the configured period, everything is fine. The
    most critical message is the heartbeat since it mostly contains static
    data. So a period shorter than the update period of the fastest static
    message is fine in any case (less than 1000 ms for 1 Hz heartbeats).

Endpoint groups:

  - Multiple endpoints can be configured to be in the same endpoin group.
    Endpoints in the same group will share the same list of connected systems.  
    When using two (or more) **parallel data links**, e.g. LTE and telemetry
    radio, the endpoint **must** be grouped on both sides. Otherwise one link
    will not be used any more because of routing rule 1.

Message Sniffing:

  - A Sniffer can be defined by setting SnifferSysID. This will forward all traffic
    to endpoints on which this MAVLink system ID is connected. This can be used to
    log or view all messages flowing though mavlink-router.

### Logging

#### Flight Stack Logging

Mavlink router can also collect flight stack logs. It supports collecting both
PX4 and Ardupilot flight stacks logs. To start logging, set a directory to the
`Log` key in the `General` section in the config file (or use argument option
`-l`). The MAVLink dialect will be auto-detected by default, or can be set
explictly using the `MavlinkDialect` key. For instance, to collect Ardupilot
logs to `/var/log/flight-stack` directory, one could add to the conf file:

    [General]
    Log=/var/log/flight-stack
    MavlinkDialect=ardupilotmega

Logs are collected on `.bin` (for Ardupilot) or `.ulg` (for PX4) files in the
specified directory. Note that they are named `XXXXX-date-time`, where `XXXXX`
is an increasing number.

#### Telemetry Logging

Similar to flight stack logging its also possible to write the raw telemetry 
data as `.tlog` file using the `LogTelemetry` key in the `General` section 
(or use argument `-T`). Note that this only works if a path using flight
stack logging is set! 
All options from flightstack logging apply also here.


## Contributing

Pull-requests are accepted on GitHub. Some guidelines to help:

1) When making changes, please follow the coding style defined by the file
   .clang-format in this repository. Coding style changes can be done automatically
   by running `ninja -C build clang-format`. Note however that there shouldn't be
   a commit on top of your changes fixing the coding style: every **commit**
   should be correct by itself.

2) Every commit should only do one thing. That is, if your work requires some
   cleaning up of code or additional fix, do that as a separate commit and not
   with your functional changes.  Find ways to take "steps" in modifying code. If
   you can break up your changes in a series of steps, do so. There may be
   exceptions to this rule, in which case it must be mentioned in the commit
   message.

3) Commit message is the best place to document your change - more than "what"
   is being done, it's important to emphasize "why" it's done. The PR description
   can describe the entire change, but the commit message is important to describe
   each step taken.

4) Communicate the state of the change: is it tested? Should it be considered
   ready to review and apply?  A few scenarios are covered by our testsuite
   by running `ninja -C build test` (the googletest library must be installed
   using your package manager first), but that doesn't replace real-world testing.


## Samples

Directory `examples` has some samples that can be used to test mavlink-router.
Those are Python scripts, and [pymavlink](https://github.com/ArduPilot/pymavlink)
is required.


### Sender & Receiver

One can test mavlink-router by using `examples/sender.py` and
`examples/receiver.py` to simulate traffic of mavlink messages. First script
sends mavlink *ping* messages to a target mavlink system-id, and second
receives and responds to them. For instance:

    $ python3 examples/sender.py 127.0.0.1:3000 100 0

Will send mavlink *pings* to UDP port 3000. Those pings will have `100` as
source system id and will have `0` as target system id (`0` means broadcast).
Receiver could be set as:

    $ python3 examples/receiver.py 127.0.0.1:4000 50

Where `50` is the receiver system id. Then, to route between those:

    $ mavlink-routerd -e 127.0.0.1:4000 0.0.0.0:3000

Note that it's possible to setup multiple senders and receivers to see
mavlink-router in action.

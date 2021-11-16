## MAVLink Router ##

Route mavlink packets between endpoints.

The usual configuration is to have one "master" endpoint that is the flight
stack (either on UART or UDP) and other components that can be on UDP or TCP
or UART endpoints. This is not strictly required and other configurations are
possible: mavlink-router mainly routes mavlink packets from one endpoint to
the other endpoints without differentiating what they are. However the
user is still able to differentiate them by targeting just a subset of the
packets by using message filters.

TCP endpoints are added automatically if the TCP server is enabled, allowing clients
to simply connect to mavlink-router without changing its configuration.

### Compilation and installation ###

In order to compile you need the following packages:

  - GCC or Clang compiler
  - meson >= 0.57 and ninja-build
  - C and C++ standard libraries

#### Fetch dependencies: ####

We currently depend on the mavlink C library. The corresponding submodule
should be fetched:

    $ git submodule update --init --recursive

We need some additional packages as build dependencies. Packages for some distros:

Debian/Ubuntu:

    $ sudo apt install git meson ninja-build pkg-config gcc g++ systemd

Fedora:

    $ sudo dnf install git meson gcc g++ systemd

Archlinux:

    $ sudo pacman -S git meson gcc systemd

#### Build ####

The build system follows the usual configure/build/install cycle. Configuration is needed
to be done only once.

A typical configuration for a x86-64 system is shown below:

    $ meson setup build .

By default systemd integration is enabled. In a system without systemd you should
pass the service directory with `-Dsystemdsystemunitdir=/usr/lib/systemd/system`
By default the build type is settled to "debugoptimized". It can be changed with
`--buildtype=release`, see [Meson documentation](https://mesonbuild.com/Builtin-options.html#core-options) for more options

Installation location can be changed using the -Dprefix option to `meson setup`.

Build:

    $ ninja -C build

Install:

    $ sudo ninja -C build install

Or in order to install to another root directory:

    $ sudo DESTDIR=/tmp/root/dir ninja -C build install

### Running ###

To route mavlink packets from master `ttyS1` to 2 other UDP endpoints, do as
following:

    $ mavlink-routerd -e 192.168.7.1:14550 -e 127.0.0.1:14550 /dev/ttyS1:1500000

The `1500000` after the colon in `/dev/ttyS1:1500000` sets the
UART baudrate. See more options with `mavlink-routerd --help`.

It's also possible to route mavlinks packets from any interface using:

    $ mavlink-routerd -e 192.168.7.1:14550 -e 127.0.0.1:14550  0.0.0.0:24550

mavlink-router also listens, by default, on port 5760 for TCP connections. Any
connection there will also receive routed packets.

IPv6 addresses must be enclosed in square brackets like this: `[::1]`. The port number
can be specified in the same way, as with IPv4 then: `[::1]:14550`.

<a name="Conffiles"></a>
### Conf file ###

It's also possible to use a .conf file to set options for mavlink-routerd.
By default, mavlink-routerd looks for a file
`/etc/mavlink-router/main.conf`. The file location can be overriden via a
`MAVLINK_ROUTER_CONF_FILE` environment variable, or via the `-c` switch when running
mavlink-routerd.
An example conf file can be found in [examples/config.sample](examples/config.sample)

#### Conf dirs ####

Besides the default conf file, it's also possible to use a directory in where to
put some extra configuration files. Files in this directory will be read in
alphabetical order, and can add or override configurations found in previous
files.

By default, `/etc/mavlink-router/config.d` is the directory, but it can be
overriden via a `MAVLINK_ROUTER_CONF_DIR` environment variable, or via the `-d`
switch when running mavlink-routerd.

Suppose that the default configuration file defines an `UartEndpoint bravo` using `Baud=56600`,
an example of overriding configuration would be:

```ini
[UartEndpoint bravo]
Baud = 115200
```

That would change `Endpoint bravo` baudrate to `115200`.

### Flight stack logging ###

Mavlink router can also collect flight stack logs. It supports collecting
both PX4 and Ardupilot flight stacks logs. To start logging, set a
directory to the `Log` key in the `General` section in the config file (or use
argument option `-l`).
Currently, mavlink router needs to be informed which MAVLink dialect the
flight stack speaks, `common` or `ardupilotmega`. To define it, use the
`MavlinkDialect` key. For instance, to collect Ardupilot logs to
`/var/log/flight-stack` directory, one could add to the conf file:

    [General]
    Log=/var/log/flight-stack
    MavlinkDialect=ardupilotmega

Logs are collected on `.bin` (for Ardupilot) or `.ulg` (for PX4) files
in the specified directory. Note that they are named `XXXXX-date-time`,
where `XXXXX` is an increasing number.

For more information about configuration files, see [conf file section](#Conffiles).

### Contributing ###

Pull-requests are accepted on GitHub. Make sure to check coding style with the
provided script in tools/checkpatch and tools/checkpython, check for memory leaks
with valgrind and test on real hardware.

### Samples ###

Directory examples has some samples that can be used to test mavlink-router.
Those are Python scripts, and [pymavlink](https://github.com/ArduPilot/pymavlink)
is required.

#### Sender & receiver ####

One can test mavlink-router by using `examples/sender.py` and
`examples/receiver.py` to simulate traffic of mavlink messages.
First script sends mavlink *ping* messages to a target mavlink system-id, and
second receives and responds to them.
For instance:

    $ python examples/sender.py 127.0.0.1:3000 100 0

Will send mavlink *pings* to UDP port 3000. Those pings will have `100` as
source system id and will have `0` as target system id (`0` means broadcast).
Receiver could be set as:

    $ python examples/receiver.py 127.0.0.1:4000 50

Where `50` is the receiver system id. Then, to route between those:

    $ mavlink-routerd -e 127.0.0.1:4000 0.0.0.0:3000

Note that it's possible to setup multiple senders and receivers to see
mavlink-router in action.

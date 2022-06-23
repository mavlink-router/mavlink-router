#
# Development environment for mavlink-routerd based on Ubuntu 20.04.
#
# Author: David Jablonski <dayjaby@gmail.com>
#

FROM ubuntu:20.04
MAINTAINER David Jablonski <dayjaby@gmail.com>

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
        git \
        ninja-build \
        pkg-config \
        gcc \
        g++ \
        python3 \
        python3-pip \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN pip3 install meson
RUN mkdir -p /home/user/mavlink-router
COPY . /home/user/mavlink-router
WORKDIR "/home/user/mavlink-router"
RUN meson setup build .
RUN ninja -C build install

FROM ubuntu:20.04
COPY --from=0 /usr/bin/mavlink-routerd /usr/bin/mavlink-routerd

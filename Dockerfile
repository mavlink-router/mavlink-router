FROM ubuntu:16.04

ARG UART_DEVICE
ARG UART_BAUDRATE
ARG UDP_OUT_IP
ARG UDP_OUT_PORT

ARG DEBIAN_FRONTEND=noninteractive


RUN apt update && apt install autoconf \
    pkg-config \
    python-dev \
    python3-dev \
    python-pip \
    python3-pip \
    libtool -y

RUN pip3 install pymavlink

WORKDIR /src

ADD . /src

RUN ./autogen.sh && ./configure CFLAGS='-g -O2' \
        --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 \
    --prefix=/usr

RUN make && make install

CMD ["bash", "-c", "mavlink-routerd $UART_DEVICE:$UART_BAUDRATE -e $UDP_OUT_IP:$UDP_OUT_PORT"]
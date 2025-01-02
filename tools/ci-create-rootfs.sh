#!/bin/bash

set -eux

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

ARCH=$1
DISTRO="ubuntu:24.04"
DISTRO_ARCH=

if [ "$ARCH" == "aarch64" ]; then
    DISTRO_ARCH="arm64v8"
elif [ "$ARCH" == "armhf" ]; then
    DISTRO_ARCH="arm32v7"
else
    DISTRO_ARCH="$ARCH"
fi

# setup binfmt_misc
# it loads the binary instead of only configuring it, so it can
# be used for subsequent calls, even if in another container
sudo podman run --rm --privileged multiarch/qemu-user-static --reset -p yes

podman build --from docker.io/$DISTRO_ARCH/$DISTRO -t $ARCH-rootfs $SCRIPT_DIR
container=$(podman run -d $ARCH-rootfs)
podman export $container

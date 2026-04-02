#!/bin/bash

set -eux

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

ARCH=$1
DISTRO="ubuntu:24.04"

{
	# setup binfmt_misc
	# it loads the binary instead of only configuring it, so it can
	# be used for subsequent calls, even if in another container
	sudo podman run --rm --privileged multiarch/qemu-user-static --reset -p yes

	podman build --arch $ARCH --from docker.io/$DISTRO -t $ARCH-rootfs $SCRIPT_DIR
	container=$(podman run --arch $ARCH -d $ARCH-rootfs)
} >&2

podman export $container

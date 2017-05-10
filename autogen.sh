#!/bin/sh

set -e

oldpwd=$(pwd)
topdir=$(dirname $0)
cd $topdir

autoreconf --force --install --symlink

libdir() {
        echo $(cd "$1/$(gcc -print-multi-os-directory)"; pwd)
}

args="\
--sysconfdir=/etc \
--localstatedir=/var \
--libdir=$(libdir /usr/lib) \
"

if [ -f "$topdir/.config.args" ]; then
    args="$args $(cat $topdir/.config.args)"
fi

if [ ! -L /bin ]; then
    args="$args \
        --with-rootprefix= \
        --with-rootlibdir=$(libdir /lib) \
        "
fi

cd $oldpwd

if [ $# -ge 1 ]; then
    cmd=$1
    shift
fi

args="$args $@"

if [ "$cmd" = "c" ]; then
        $topdir/configure CFLAGS='-g -O2' CXXFLAGS='-g -O2' $args
        make clean
elif [ "$cmd" = "g" ]; then
        $topdir/configure CFLAGS='-g -Og' CXXFLAGS='-g -O2' $args
        make clean
elif [ "$cmd" = "l" ]; then
        $topdir/configure CC=clang CXX=clang++ CFLAGS='-g -O2' CXXFLAGS='-g -O2' $args
        make clean
elif [ "$cmd" = "a" ]; then
        $topdir/configure CFLAGS='-g -O2 -Wsuggest-attribute=pure -Wsuggest-attribute=const' CXXFLAGS='-g -O2 -Wsuggest-attribute=pure -Wsuggest-attribute=const' $args
        make clean
elif [ "$cmd" = "s" ]; then
        scan-build $topdir/configure CFLAGS='-g -O0 -std=gnu11' CXXFLAGS='-g -O0 -std=gnu++11' $args
        scan-build make
else
        echo
        echo "----------------------------------------------------------------"
        echo "Initialized build system. For a common configuration please run:"
        echo "----------------------------------------------------------------"
        echo
        echo "$topdir/configure CFLAGS='-g -O2' CXXFLAGS='-g -O2' $args"
        echo
fi

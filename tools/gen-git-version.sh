#!/bin/bash

set -e

dir=$1
GIT_VERSION=

cd $dir
if [ -e .git ]; then
    GIT_VERSION=\"$(git describe --tags --dirty=+ --match 'v*' 2>/dev/null)\"
fi

if [ -z "$GIT_VERSION" ]; then
    GIT_VERSION="VERSION"
fi

cat - <<-EOF
#pragma once

#define GIT_VERSION $GIT_VERSION
EOF

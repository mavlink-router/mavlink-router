#!/bin/bash

set -e

dir=$1
GIT_VERSION=

cd $dir
if [ -e .git ]; then
    # if repo is cloned with something like --depth=1 we may not have any tag
    GIT_VERSION=$(git describe --tags --dirty=+ --match 'v*' 2>/dev/null) || true
    if [ -z "$GIT_VERSION" ]; then
        GIT_VERSION="$(git rev-parse --short HEAD)" || true
    fi

    if [ -z "$GIT_VERSION" ]; then
        GIT_VERSION="VERSION"
    fi
fi

if [ -z "$GIT_VERSION" ]; then
    GIT_VERSION="VERSION"
fi

echo $GIT_VERSION
